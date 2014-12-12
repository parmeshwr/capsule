/*
 * EFI capsule support.
 *
 * Copyright 2013 Intel Corporation <matt.fleming@intel.com>
 *
 * This file is part of the Linux kernel, and is made available under
 * the terms of the GNU General Public License version 2.
 */

#define pr_fmt(fmt) "efi-capsule: " fmt

#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/highmem.h>
#include <linux/efi.h>

typedef struct {
	u64 length;
	u64 data;
} efi_capsule_block_desc_t;

static bool capsule_pending;
static int efi_reset_type = -1;

/*
 * capsule_mutex serialises access to both 'capsule_pending' and
 * 'efi_reset_type'.
 *
 * This mutex must be held across calls to efi_capsule_supported() and
 * efi_update_capsule() so that the operation is atomic. This ensures
 * that efi_update_capsule() isn't called with a capsule that requires a
 * different reset type to the registered 'efi_reset_type'.
 */
static DEFINE_MUTEX(capsule_mutex);

static int efi_update_capsule(efi_capsule_header_t *capsule,
			      struct page **pages, size_t size, int reset);

/**
 * efi_capsule_pending - has a capsule been passed to the firmware?
 * @reset_type: store the type of EFI reset if capsule is pending
 *
 * To ensure that the registered capsule is processed correctly by the
 * firmware we need to perform a specific type of reset. If a capsule is
 * pending return the reset type in @reset_type.
 */
bool efi_capsule_pending(int *reset_type)
{
	bool rv = false;

	mutex_lock(&capsule_mutex);
	if (!capsule_pending)
		goto out;

	if (reset_type)
		*reset_type = efi_reset_type;
	rv = true;

out:
	mutex_unlock(&capsule_mutex);
	return rv;
}

/**
 * efi_capsule_supported - does the firmware support the capsule?
 * @guid: vendor guid of capsule
 * @flags: capsule flags
 * @size: size of capsule data
 * @reset: the reset type required for this capsule
 *
 * Check whether a capsule with @flags is supported and that @size
 * doesn't exceed the maximum size for a capsule.
 */
int efi_capsule_supported(efi_guid_t guid, u32 flags, size_t size, int *reset)
{
	efi_capsule_header_t *capsule;
	efi_status_t status;
	u64 max_size;
	int rv = 0;

	capsule = kmalloc(sizeof(*capsule), GFP_KERNEL);
	if (!capsule)
		return -ENOMEM;

	capsule->headersize = capsule->imagesize = sizeof(*capsule);
	memcpy(&capsule->guid, &guid, sizeof(efi_guid_t));
	capsule->flags = flags;

	status = efi.query_capsule_caps(&capsule, 1, &max_size, reset);
	if (status != EFI_SUCCESS) {
		rv = efi_status_to_err(status);
		goto out;
	}

	if (size > max_size)
		rv = -ENOSPC;
out:
	kfree(capsule);
	return rv;
}

/**
 * efi_capsule_update - send a capsule to the firmware
 * @capsule:
 * @pages:
 */
int efi_capsule_update(efi_capsule_header_t *capsule, struct page **pages)
{
	efi_guid_t guid = capsule->guid;
	size_t size = capsule->imagesize;
	u32 flags = capsule->flags;
	int rv, reset_type;

	mutex_lock(&capsule_mutex);
	rv = efi_capsule_supported(guid, flags, size, &reset_type);
	if (rv)
		goto out;

	if (efi_reset_type >= 0 && efi_reset_type != reset_type) {
		pr_err("Incompatible capsule reset type %d\n", reset_type);
		rv = -EINVAL;
		goto out;
	}

	rv = efi_update_capsule(capsule, pages, size, reset_type);
out:
	mutex_unlock(&capsule_mutex);
	return rv;
}
EXPORT_SYMBOL_GPL(efi_capsule_update);

/**
 * efi_capsule_build - alloc capsule and send to firmware
 * @guid: guid of the capsule
 * @size: size in bytes of the capsule data
 *
 * This is a helper function for allocating enough room for user data
 * + the size of an EFI capsule header, and passing that capsule to the
 * firmware.
 *
 * We also atomically update the EFI reset type.
 *
 * Returns a pointer to the capsule on success, an ERR_PTR() value on
 * error. If an error is returned we guarantee that the capsule has not
 * been passed to the firmware.
 */
efi_capsule_header_t *efi_capsule_build(efi_guid_t guid, size_t size)
{
	efi_capsule_header_t *capsule = NULL;
	unsigned int nr_pages = 0;
	size_t capsule_size;
	struct page **pages;
	int i, rv = -ENOMEM;
	u32 flags = EFI_CAPSULE_PERSIST_ACROSS_RESET |
		EFI_CAPSULE_POPULATE_SYSTEM_TABLE;

	capsule_size = size + sizeof(*capsule);

	nr_pages = ALIGN(capsule_size, PAGE_SIZE) >> PAGE_SHIFT;
	pages = kzalloc(nr_pages * sizeof(void *), GFP_KERNEL);
	if (!pages)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < nr_pages; i++) {
		struct page *page;

		page = alloc_page(GFP_KERNEL);
		if (!page)
			goto fail;

		pages[i] = page;
	}

	capsule = vmap(pages, nr_pages, 0, PAGE_KERNEL);
	if (!capsule)
		goto fail;

	/*
	 * Setup the EFI capsule header.
	 */
	memcpy(&capsule->guid, &guid, sizeof(guid));

	capsule->headersize = sizeof(*capsule);
	capsule->imagesize = capsule_size;
	capsule->flags = flags;

	rv = efi_capsule_update(capsule, pages);
	if (rv)
		goto fail;
out:
	kfree(pages);
	return capsule;

fail:
	vunmap(capsule);
	for (i = 0; i < nr_pages; i++) {
		if (!pages[i])
			break;

		__free_page(pages[i]);
	}
	capsule = ERR_PTR(rv);
	goto out;
}
EXPORT_SYMBOL_GPL(efi_capsule_build);

static efi_capsule_header_t *
__map_capsule(efi_capsule_header_t *phys, efi_guid_t guid)
{
	efi_capsule_header_t *virt;
	size_t size;
	void *err = ERR_PTR(-ENOMEM);

	virt = ioremap((resource_size_t)phys, sizeof(*virt));
	if (!virt) {
		pr_err("failed to ioremap capsule\n");
		goto fail;
	}

	size = virt->imagesize;
	iounmap(virt);

	virt = ioremap((resource_size_t)phys, size);
	if (!virt) {
		pr_err("failed to ioremap header + data\n");
		goto fail;
	}

	/*
	 * This *really* shouldn't happen, since the firmware groups all
	 * capsules with the same guid together.
	 */
	if (unlikely(efi_guidcmp(virt->guid, guid))) {
		pr_err("guid mismatch inside capsule\n");
		iounmap(virt);
		err = ERR_PTR(-EINVAL);
		goto fail;
	}

	return virt;
fail:
	return err;
}

static efi_guid_t mappings[] = {
	LINUX_EFI_CRASH_GUID,
	LINUX_EFI_BLK_DEV_GUID,
};

/**
 * efi_capsule_lookup - search capsule array for entries.
 * @guid: the guid to search for.
 * @nr_found: the number of entries found.
 *
 * Map each capsule header into the kernel's virtual address space and
 * inspect the guid. Build an array of capsule headers with every
 * capsule that is found with @guid. If a match is found the capsule
 * remains mapped, otherwise it is unmapped.
 *
 * This function searches the capsule array built at efi_config_init()
 * time for capsules matching @guid. If we find a matching array
 * we remap each capsule into the kernel's virtual address space and
 * return a new array of virtually mapped capsule pointers.
 *
 * Returns an array of capsule headers, each element of which has the
 * guid @guid. The number of elements in the array is stored in
 * @nr_found. Returns %NULL and stores zero in @nr_found if no capsules
 * were found.
 *
 * If capsules were found but an error condition was encountered when
 * retrieving them, an ERR_PTR() value is returned instead of NULL.
 */
efi_capsule_header_t **efi_capsule_lookup(efi_guid_t guid, uint32_t *nr_found)
{
	efi_capsule_header_t **capsules = NULL;
	efi_capsule_header_t **cap_array;
	uint32_t nr_capsules;
	int i;

	*nr_found = 0;

	for (i = 0; i < EFI_LINUX_CAPSULES_NR; i++) {
		unsigned long addr;
		void *cap_header;
		size_t size;

		if (efi_guidcmp(mappings[i], guid))
			continue;

		if (!efi.capsules[i])
			continue;

		addr = efi.capsules[i];
		cap_header = ioremap(addr, sizeof(nr_capsules));
		if (!cap_header)
			return ERR_PTR(-ENOMEM);

		/*
		 * The array of capsules is prefixed with the number of
		 * capsule entries in the array for this guid.
		 */
		nr_capsules = *(uint32_t *)cap_header;
		iounmap(cap_header);

		/*
		 * This shouldn't happen. If it does, it likely indicates
		 * buggy firmware since it is the firmware that writes
		 * this value.
		 */
		if (!nr_capsules) {
			pr_err("capsule contains no entries\n");
			continue;
		}

		/* Allocate up front */
		capsules = kmalloc(nr_capsules * sizeof(*capsules), GFP_KERNEL);
		if (!capsules)
			goto fail;

		size = nr_capsules * sizeof(*cap_array) + sizeof(nr_capsules);
		cap_header = ioremap(addr, size);
		if (!cap_header)
			goto fail;

		cap_array = cap_header + sizeof(uint32_t *);

		for (i = 0; i < nr_capsules; i++) {
			efi_capsule_header_t *c;

			c = __map_capsule(cap_array[i], guid);
			if (IS_ERR(c)) {
				pr_err("Failed to map capsule %d guid %pUl\n",
				       i, guid.b);
				break;
			}
			capsules[i] = c;
			*nr_found += 1;
		}

		iounmap(cap_header);

		/*
		 * Exit early. We know there are no other matches in the
		 * capsule list because they're grouped by guid.
		 */
		break;
	}

	return capsules;

fail:
	kfree(capsules);
	return ERR_PTR(-ENOMEM);
}
EXPORT_SYMBOL_GPL(efi_capsule_lookup);

#define BLOCKS_PER_PAGE	(PAGE_SIZE / sizeof(efi_capsule_block_desc_t))

/*
 * How many pages of block descriptors do we need to map 'nr_pages'?
 *
 * Every list of block descriptors in a page must end with a
 * continuation pointer. The last continuation pointer of the lage page
 * must be zero to mark the end of the chain.
 */
static inline unsigned int num_block_pages(unsigned int nr_pages)
{
	return DIV_ROUND_UP(nr_pages, BLOCKS_PER_PAGE - 1);
}

/**
 * efi_update_capsule - pass a single capsule to the firmware.
 * @capsule: capsule to send to the firmware.
 * @pages: an array of capsule data.
 * @size: total size of capsule data + headers in @capsule.
 * @reset: the reset type required for @capsule
 *
 * Map @capsule with EFI capsule block descriptors in PAGE_SIZE chunks.
 * @size needn't necessarily be a multiple of PAGE_SIZE - we can handle
 * a trailing chunk that is smaller than PAGE_SIZE.
 *
 * @capsule MUST be virtually contiguous.
 *
 * Return 0 on success.
 */
static int efi_update_capsule(efi_capsule_header_t *capsule,
			      struct page **pages, size_t size, int reset)
{
	efi_capsule_block_desc_t *block = NULL;
	struct page **block_pgs;
	efi_status_t status;
	unsigned int nr_data_pgs, nr_block_pgs;
	int i, j, err = -ENOMEM;

	nr_data_pgs = DIV_ROUND_UP(size, PAGE_SIZE);
	nr_block_pgs = num_block_pages(nr_data_pgs);

	block_pgs = kzalloc(nr_block_pgs * sizeof(*block_pgs), GFP_KERNEL);
	if (!block_pgs)
		return -ENOMEM;

	for (i = 0; i < nr_block_pgs; i++) {
		block_pgs[i] = alloc_page(GFP_KERNEL);
		if (!block_pgs[i])
			goto fail;
	}

	for (i = 0; i < nr_block_pgs; i++) {
		block = kmap(block_pgs[i]);
		if (!block)
			goto fail;

		for (j = 0; j < BLOCKS_PER_PAGE - 1 && nr_data_pgs > 0; j++) {
			u64 sz = min_t(u64, size, PAGE_SIZE);

			block[j].length = sz;
			block[j].data = page_to_phys(*pages++);

			size -= sz;
			nr_data_pgs--;
		}

		/* Continuation pointer */
		block[j].length = 0;

		if (i + 1 == nr_block_pgs)
			block[j].data = 0;
		else
			block[j].data = page_to_phys(block_pgs[i + 1]);

		kunmap(block_pgs[i]);
	}

	status = efi.update_capsule(&capsule, 1, page_to_phys(block_pgs[0]));
	if (status != EFI_SUCCESS) {
		pr_err("update_capsule fail: 0x%lx\n", status);
		err = efi_status_to_err(status);
		goto fail;
	}

	capsule_pending = true;
	efi_reset_type = reset;

	kfree(block_pgs);
	return 0;

fail:
	for (i = 0; i < nr_block_pgs; i++) {
		if (block_pgs[i])
			__free_page(block_pgs[i]);
	}

	kfree(block_pgs);
	return err;
}
