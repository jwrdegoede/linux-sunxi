/*
 * NAND Flash partitions described by the OF (or flattened) device tree
 *
 * Copyright © 2014 Boris BREZILLON <b.brezillon.dev@gmail.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/mtd/mtd.h>
#include <linux/slab.h>
#include <linux/mtd/nand.h>

static inline bool node_has_compatible(struct device_node *pp)
{
	return of_get_property(pp, "compatible", NULL);
}

int ofnandpart_parse(struct mtd_info *master,
		     const struct ofnandpart_data *data)
{
	struct device_node *node;
	const char *partname;
	struct device_node *pp;
	int i;

	if (!data)
		return 0;

	node = data->node;
	if (!node)
		return 0;

	i = 0;
	for_each_child_of_node(node,  pp) {
		const __be32 *reg;
		int len;
		int a_cells, s_cells;
		uint64_t offset, size;
		uint32_t mask_flags = 0;
		struct nand_part *part;

		if (node_has_compatible(pp))
			continue;

		reg = of_get_property(pp, "reg", &len);
		if (!reg)
			continue;

		a_cells = of_n_addr_cells(pp);
		s_cells = of_n_size_cells(pp);
		offset = of_read_number(reg, a_cells);
		size = of_read_number(reg + a_cells, s_cells);

		partname = of_get_property(pp, "label", &len);
		if (!partname)
			partname = of_get_property(pp, "name", &len);

		if (of_get_property(pp, "read-only", &len))
			mask_flags |= MTD_WRITEABLE;

		if (of_get_property(pp, "lock", &len))
			mask_flags |= MTD_POWERUP_LOCK;

		if (data->parse)
			part = data->parse(data->priv, master, pp);
		else
			part = nandpart_alloc();

		if (IS_ERR(part))
			continue;

		part->offset = offset;
		part->master = master;
		part->mtd.name = partname;
		part->mtd.size = size;
		part->mtd.flags = mask_flags;

		if (nand_add_partition(master, part)) {
			if (part->release)
				part->release(part);
			continue;
		}

		i++;
	}

	if (!i) {
		of_node_put(pp);
		pr_err("No valid partition found on %s\n", node->full_name);
	}

	return i;
}
EXPORT_SYMBOL(ofnandpart_parse);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Parser for NAND flash partitioning information in device tree");
MODULE_AUTHOR("Boris BREZILLON");
