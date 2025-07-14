# PATCH

PATCH模块可用于修补现有的LCPU ROM数据/代码。 每个PATCH条目都可以替换ROM空间中的4个字节数据（地址需要为4字节对齐）。SIFLI芯片组在LCPU中具有ROM代码，其中包含BLE堆栈，操作系统和其他有用的功能。 如果这些代码有错误，则使用PATCH来修复它们。 当LCPU睡觉时，那些补丁设置需要保存到AON（始终打开）内存，并在LCPU从睡眠中唤醒时再次应用这些补丁。


## 使用PATCH
以下代码将保存和应用补丁。 

```c

/*Power on or wake up*/
struct patch_entry_desc g_lcpu_patch_list[]= { 
    { 0x0000DC14, 0xB91CF110 },
	{ 0x0000DEA4, 0xB8F2F110 } 
};
HAL_PATCH_install((struct patch_entry_desc *)g_lcpu_patch_list(sizeof(g_lcpu_patch_list))/sizeof(struct patch_entry_desc));

..
/*before sleep, g_lcpu_patch_list should in AON memory section*/
uint32_t cer;
HAL_PATCH_save(g_lcpu_patch_list, g_lcpu_patch_list(sizeof(g_lcpu_patch_list))/sizeof(struct patch_entry_desc), &cer);

..

```

## API参考
[](../api/hal/patch.md)

