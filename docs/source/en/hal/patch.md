# PATCH

The PATCH module can be used to patch existing LCPU ROM data/code. Each PATCH
entry can replace 4 bytes of data in ROM space (address must be 4-byte aligned).
SIFLI chipsets have ROM code in LCPU that contains BLE stack, operating system,
and other useful functions. If these codes have bugs, PATCH is used to fix them.
When LCPU sleeps, those patch settings need to be saved to AON (Always On)
memory and reapplied when LCPU wakes up from sleep.


## Using PATCH
The following code will save and apply patches.

```c
/*Power on or wake up*/
struct patch_entry_desc g_lcpu_patch_list[]= { 
    { 0x0000DC14, 0xB91CF110 },
	{ 0x0000DEA4, 0xB8F2F110 } 
};
HAL_PATCH_install((struct patch_entry_desc *)g_lcpu_patch_list, sizeof(g_lcpu_patch_list) / sizeof(struct patch_entry_desc));

..
/*Before entering sleep mode, ensure g_lcpu_patch_list is located in the AON memory section[X266X]*/
uint32_t cer;
HAL_PATCH_save(g_lcpu_patch_list, sizeof(g_lcpu_patch_list) / sizeof(struct patch_entry_desc), &amp;cer);

..
```

## API Reference
[]

