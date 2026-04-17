#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif



static const char ____versions[]
__used __section("__versions") =
	"\x14\x00\x00\x00\x44\x43\x96\xe2"
	"__wake_up\0\0\0"
	"\x24\x00\x00\x00\x93\x23\x8e\xda"
	"__platform_driver_register\0\0"
	"\x24\x00\x00\x00\x68\x1c\x6e\x6e"
	"platform_driver_unregister\0\0"
	"\x18\x00\x00\x00\x2b\x40\x05\xc9"
	"log_write_mmio\0\0"
	"\x1c\x00\x00\x00\x24\x3c\x4d\xfa"
	"log_post_write_mmio\0"
	"\x18\x00\x00\x00\xc0\xc5\x82\xd7"
	"misc_deregister\0"
	"\x1c\x00\x00\x00\x37\xc8\x98\xe4"
	"dma_release_channel\0"
	"\x14\x00\x00\x00\x9d\xd9\xe6\xb6"
	"clk_disable\0"
	"\x18\x00\x00\x00\x0a\xe7\x77\xb0"
	"clk_unprepare\0\0\0"
	"\x18\x00\x00\x00\x6f\xfa\x7f\xdc"
	"devm_iounmap\0\0\0\0"
	"\x20\x00\x00\x00\x85\xa4\xf3\x6f"
	"dynamic_might_resched\0\0\0"
	"\x24\x00\x00\x00\x75\x08\x94\x89"
	"mutex_lock_interruptible\0\0\0\0"
	"\x18\x00\x00\x00\x38\xf0\x13\x32"
	"mutex_unlock\0\0\0\0"
	"\x18\x00\x00\x00\x75\x79\x48\xfe"
	"init_wait_entry\0"
	"\x14\x00\x00\x00\x51\x0e\x00\x01"
	"schedule\0\0\0\0"
	"\x20\x00\x00\x00\x95\xd4\x26\x8c"
	"prepare_to_wait_event\0\0\0"
	"\x14\x00\x00\x00\xbf\x0f\x54\x92"
	"finish_wait\0"
	"\x1c\x00\x00\x00\xcb\xf6\xfd\xf0"
	"__stack_chk_fail\0\0\0\0"
	"\x10\x00\x00\x00\xad\x64\xb7\xdc"
	"memset\0\0"
	"\x20\x00\x00\x00\x28\xe1\xa4\x12"
	"__arch_copy_from_user\0\0\0"
	"\x1c\x00\x00\x00\xef\x6d\x5c\xa6"
	"alt_cb_patch_nops\0\0\0"
	"\x24\x00\x00\x00\x52\x3f\x0a\x4b"
	"gic_nonsecure_priorities\0\0\0\0"
	"\x1c\x00\x00\x00\x9a\xe6\x97\xd6"
	"trace_hardirqs_on\0\0\0"
	"\x1c\x00\x00\x00\x1b\x2e\x3d\xec"
	"trace_hardirqs_off\0\0"
	"\x18\x00\x00\x00\x80\x21\x72\x6b"
	"log_read_mmio\0\0\0"
	"\x1c\x00\x00\x00\xe3\x5e\x1a\x0c"
	"log_post_read_mmio\0\0"
	"\x1c\x00\x00\x00\x54\xfc\xbb\x6c"
	"__arch_copy_to_user\0"
	"\x28\x00\x00\x00\xb3\x1c\xa2\x87"
	"__ubsan_handle_out_of_bounds\0\0\0\0"
	"\x2c\x00\x00\x00\x61\xe5\x48\xa6"
	"__ubsan_handle_shift_out_of_bounds\0\0"
	"\x20\x00\x00\x00\x10\xa6\x07\x0d"
	"platform_get_resource\0\0\0"
	"\x20\x00\x00\x00\xef\x89\x70\xa6"
	"devm_ioremap_resource\0\0\0"
	"\x18\x00\x00\x00\x2d\xc8\x8c\xb2"
	"devm_clk_get\0\0\0\0"
	"\x14\x00\x00\x00\x71\x73\x9a\x7c"
	"clk_prepare\0"
	"\x14\x00\x00\x00\x37\x5f\x97\x2a"
	"_dev_err\0\0\0\0"
	"\x14\x00\x00\x00\xa6\x88\x55\x81"
	"clk_enable\0\0"
	"\x1c\x00\x00\x00\x2f\x71\x1b\x33"
	"dma_request_chan\0\0\0\0"
	"\x18\x00\x00\x00\x75\x19\x4b\x3f"
	"dma_set_mask\0\0\0\0"
	"\x18\x00\x00\x00\x75\xc4\xf8\x24"
	"misc_register\0\0\0"
	"\x1c\x00\x00\x00\xfc\xd8\x29\x6c"
	"pin_user_pages_fast\0"
	"\x2c\x00\x00\x00\x9c\x8e\xdd\x3c"
	"sg_alloc_table_from_pages_segment\0\0\0"
	"\x18\x00\x00\x00\xeb\x26\x5f\xa9"
	"dma_map_sgtable\0"
	"\x1c\x00\x00\x00\x00\x06\x9f\x87"
	"dma_unmap_sg_attrs\0\0"
	"\x18\x00\x00\x00\x86\x50\xc8\xc8"
	"sg_free_table\0\0\0"
	"\x1c\x00\x00\x00\xe4\xdf\x5d\x61"
	"unpin_user_pages\0\0\0\0"
	"\x18\x00\x00\x00\x78\x86\x54\x3e"
	"param_ops_int\0\0\0"
	"\x18\x00\x00\x00\xfb\x29\xf6\xc8"
	"module_layout\0\0\0"
	"\x00\x00\x00\x00\x00\x00\x00\x00";

MODULE_INFO(depends, "");

MODULE_ALIAS("of:N*T*Crp1-ws281x-pwm");
MODULE_ALIAS("of:N*T*Crp1-ws281x-pwmC*");

MODULE_INFO(srcversion, "DE44F38DD44C8A08CA2333C");
