# LVGL FreeType Fonts

SiFli SDK integrates a FreeType font adapter and the LVSF font manager with LVGL v8. Applications normally store complete TTF files as ordinary files in a mounted RT-Thread DFS file system and load them by path with `lvsf_font_load_ex()`; no conversion to C arrays is required. The font manager creates and caches an `lv_font_t` for each requested pixel size, ready for use by LVGL widgets and styles.

Vector fonts are useful for interfaces that require multiple sizes, large character sets, or runtime font switching. Compared with pregenerated bitmap fonts, FreeType requires additional memory for font parsing, glyph rendering, and caching.

- Main headers: _middleware/lvgl/lvsf/lvsf_font.h_, _middleware/lvgl/lvsf/lvsf_font_manager.h_
- Registration descriptors: _middleware/lvgl/lvsf/lvsf_ft_reg.h_
- Implementation directories: _middleware/lvgl/lvsf/_, _external/freetype/_

```{note}
This document covers the SiFli font APIs selected by `LV_USING_FREETYPE_ENGINE` and applies only to LVGL v8. The upstream LVGL integration selected by `LV_USE_FREETYPE` uses APIs such as `lv_ft_font_init()` and does not include the LVSF font manager. Do not mix the two API sets.
```

## System Components

| Component | Responsibility |
| --- | --- |
| FreeType | Opens font sources, reads glyph outlines, and generates grayscale glyph bitmaps |
| SiFli font adapter | Wraps a font source at a specific pixel size in an LVGL `lv_font_t` |
| LVSF font manager | Registers font entries and creates, caches, selects, and releases managed font objects |
| LVGL | Stores and uses `lv_font_t` pointers in widgets, styles, and themes |

The font manager first stores a font entry, then creates one `lv_font_t` for each combination of font entry and pixel size. While that object remains cached, repeated requests for the same combination return the same pointer. If a cache-clear or unload operation actually releases the object, the original pointer becomes invalid.

Pointers returned by the font manager are borrowed. Do not free them or pass them to `lv_freetype_font_deinit()` or `lv_freetype_set_font_size()`. Applications may set `fallback`, but must follow the reference and lifetime requirements described below.

### Font Sources

For applications that already use a file system, loading a complete TTF file at runtime is the most direct approach. Build-time embedding and runtime memory registration are intended for fonts that must not depend on a file system or whose data must remain resident.

The time at which a font entry is registered is independent of how FreeType opens its font source. A statically registered entry can refer to either an array embedded in firmware or a fixed file path, while a descriptor registered at runtime can refer to in-memory data.

Here, "embedded at build time" means only that the font source is linked into the firmware; its outlines are not preconverted to an LVGL bitmap font. Whether the source comes from firmware, a file, or in-memory data registered at runtime, the font manager creates each size-specific `lv_font_t` at runtime, and FreeType parses and renders the font's glyphs.

| Use case | Registration API | Managed-object creation | Font source lifetime | Unregistration through `lvsf_font_unload_ex()` |
| --- | --- | --- | --- | --- |
| Complete TTF file in a file system (common) | `lvsf_font_load_ex()` | Immediately attempts to create the requested sizes during registration | File must remain readable until the font is unloaded | Yes, unless the path was previously registered as a non-unloadable entry |
| DFS font file registered for lazy loading | `lvsf_font_register()` with `external=1` | Created on first lookup | File must remain readable until the font is unloaded | Yes, unless the path was previously registered as a non-unloadable entry |
| Font array embedded at build time (optional) | `LVSF_FREETYPE_FONT_REGISTER()` | Common sizes for the default entry may be created at startup; other sizes are created on demand | Entire firmware lifetime | No; only its managed font cache can be cleared |
| Fixed file path registered at build time (optional) | `LVSF_FREETYPE_FONT_REGISTER()` with `font_lib_size=0` | Common sizes for the default entry may be created at startup; other sizes are created on demand | File must remain readable for the entire firmware run | No; only its managed font cache can be cleared |
| In-memory descriptor registered at runtime (optional) | `lvsf_font_register_lib()` | Created on first lookup | Descriptor and data must remain valid while the font manager can use them | No; only its managed font cache can be cleared |

`font_lib_size` determines how FreeType opens the font:

- When `font_lib_size > 0`, `font_lib_data` points to font data in memory.
- When `font_lib_size == 0`, `font_lib_data` is interpreted as a file path.

This distinction does not determine whether `lvsf_font_unload_ex()` can unregister the entry. Unloading support depends on the type of entry created by the registration API.

For the two file registration APIs — `lvsf_font_load_ex()` and `lvsf_font_register()` — the path is the entry reuse key: registering the same path again reuses the entry created by the first registration and does not change its lookup name or source attributes, including whether it is unloadable. `lvsf_font_register_lib()` and linker-section registration do not deduplicate by path — the former reuses by descriptor pointer or name, the latter only by descriptor pointer — so registering an already-registered file path through them creates a second entry sharing that path. Do not mix linker-section registration, `lvsf_font_register_lib()`, and runtime file registration for the same font path.

```{warning}
When the current manager creates a linker-section entry or an entry from `lvsf_font_register_lib()`, it copies from `font_lib_data` into the entry's `path` field using C string semantics even if `font_lib_size > 0`. Every in-memory font buffer must contain complete, trusted TrueType data and must be safe to read through a `\0` byte within its valid bounds. Do not register unvalidated, truncated, or short-lived download buffers. This is a limitation of the current implementation, not a FreeType requirement for in-memory fonts.
```

## Configuration and Initialization

### Enabling the Component

In `menuconfig`, select `Third-Party Components -> LittlevGL2RTT -> LVGL configuration -> SiFli extend` and enable the FreeType font engine.

```none
CONFIG_LV_USING_FREETYPE_ENGINE=y
CONFIG_LV_USE_USER_DATA=y
```

Loading fonts from a file system also requires RT-Thread DFS and the file-system implementation for the storage medium:

```none
CONFIG_RT_USING_DFS=y
```

FreeType itself does not require `RT_USING_DFS_ELMFAT`. Enable it only in projects that use the ELM FAT file system.

The current font adapter registers only the TrueType outline driver. This restriction applies to both file-based and in-memory fonts; a file extension alone does not determine compatibility. OpenType fonts with CFF outlines must first be converted to TrueType outlines.

```{warning}
The implementation described here requires the LVGL v8 FreeType integration. Enabling `PKG_SCHRIFT` or `USING_VGLITE` together with `LV_USING_FREETYPE_ENGINE` does not build: the body of _lv_freetype.c_ is excluded by conditional compilation while the font manager still compiles and references these APIs. Do not combine either setting with the APIs described here.
```

### Initialization

The standard SDK startup sequence initializes FreeType and the font manager through `gui_lib_init()`. Applications normally do not need to initialize them again.

If a project bypasses the standard component initialization sequence, call the manager initialization function and check its return value:

```c
static int init_font_manager(void)
{
    return lvsf_font_manager_init(0);
}
```

With an argument of 0, the manager uses the FreeType cache capacity provided by the SDK. Repeated calls do not rebuild an already initialized FreeType cache.

The global font registry, managed font cache, and FreeType FTC cache have no internal locking. After the GUI starts, serialize font registration, lookup, ordering, cache clearing, and unloading in the LVGL thread. Do not modify font state concurrently with rendering.

## Loading and Registering Fonts

### Loading a Complete TTF File from a File System

This is the common way to use LVSF FreeType fonts. The project deploys the complete TTF file as an ordinary file to its chosen storage medium and mounts the corresponding RT-Thread DFS file system. The font does not need to be converted to a C array or linked into the application firmware. The project's resource deployment process determines how the file reaches storage; LVSF only requires the file to be readable through a DFS path before the API is called.

`lvsf_font_load_ex()` passes the DFS path to FreeType to open the font. This is not an LVGL file-system drive path.

```c
#include "lvsf/lvsf_font.h"
#include "lvsf/lvsf_font_manager.h"

#define APP_FONT_PATH "/data/fonts/MyFont.ttf"
#define APP_FONT_NAME "MyFont"

static int load_app_font(void)
{
    uint16_t sizes[] = {16, 24, 32, 0};

    if (lvsf_font_load_ex(APP_FONT_PATH, sizes) != 1)
    {
        return -1;
    }

    return lvsf_font_get(APP_FONT_NAME, 24) != NULL ? 0 : -1;
}
```

The `sizes` array must end with 0. Passing `NULL` creates only the size represented by `FONT_NORMAL`; `{0}` contains no valid size and causes loading to fail.

`lvsf_font_load_ex()` returns 1 if at least one requested size is created successfully. It returns -1 if the path is invalid, entry registration fails, or every requested size fails. A successful batch load does not guarantee that every size is available. Retrieve each size the application needs and check the result.

The manager derives the lookup name from the final path component after removing its last extension. The path `/data/fonts/MyFont.v2.ttf` therefore maps to `MyFont.v2`. Names are case-sensitive. If different paths produce the same name, the later registration is rejected. Loading the same path again reuses the original entry and preserves the name and source attributes established by the first registration.

The FreeType cache may reopen the font file later. Do not delete or replace the file, or unmount the file system that contains it, immediately after `lvsf_font_load_ex()` returns. The file must remain readable until the corresponding font entry has been unloaded successfully.

### Optional: Registering a File for Lazy Loading

`lvsf_font_register()` registers a file path and its selection attributes without validating the file until the first lookup. It can assign a lookup name to a previously unregistered path or configure an entry's enabled state and priority before creating any managed font object.

```c
static const lvsf_font_config_t app_font_config =
{
    .name = "AppBody",
    .path = "/data/fonts/MyFont.ttf",
    .external = 1,
    .enabled = 1,
    .priority = 10,
};

static int register_app_font(void)
{
    return lvsf_font_register(&app_font_config);
}
```

For a path that has not been registered, `external=1` creates a runtime file entry that `lvsf_font_unload_ex()` can unregister. If the path already exists, the API reuses the original entry and preserves its name and source attributes. `external=0` does not create a new in-memory font; it can only configure an existing entry. Setting `priority=0` also makes the entry the current default.

`lvsf_font_register_batch()` processes its array in order and stops at the first error. It does not roll back entries registered successfully before that error. Applications that require an atomic update must track and handle those entries themselves.

### Optional: Embedding a Font at Build Time

When no file system is available, or the font data must remain resident with the firmware, convert the complete TrueType file to a read-only C array. This changes only where the font data is stored; FreeType still parses the font and renders glyphs at runtime.

The generated header declares the array and its actual byte count:

```c
/* app_font_data.h */
#include <stdint.h>

#define APP_FONT_DATA_SIZE 12345U
extern const uint8_t app_font_data[APP_FONT_DATA_SIZE];
```

Then define the font library descriptor and register it in the `app_font` linker section:

```c
#include <stdint.h>
#include "lvsf/lvsf_ft_reg.h"
#include "app_font_data.h"

const lv_font_freetype_lib_dsc_t AppFont_lib =
{
    .font_lib_size = APP_FONT_DATA_SIZE,
    .font_lib_data = (const char *)app_font_data,
    .font_lib_name = "AppFont",
};

LVSF_FREETYPE_FONT_REGISTER(AppFont);
```

`LVSF_FREETYPE_FONT_REGISTER(AppFont)` requires a descriptor named `AppFont_lib` with external linkage. During standard startup, the manager scans the linker section and creates the font entry. The current manager derives the lookup name from the macro argument `AppFont`, so applications retrieve the font with `lvsf_font_get("AppFont", size)`. The `font_lib_name` member of the descriptor is not currently used for name lookup.

Linker-section font entries are enabled by default and have priority 0. The first previously unregistered font library descriptor encountered while scanning the section becomes the initial built-in entry and default entry. Size records in the linker section do not prevent the current manager from creating other pixel sizes on demand.

The embedded array and its descriptor must remain valid for the entire firmware run. `lvsf_font_unload_ex()` cannot unregister linker-section entries. When none of the font's created sizes are needed, replace every reference to those sizes and then call `lvsf_font_clear_cache("AppFont")`. There is currently no API for clearing only one size.

### Optional: Registering an In-Memory Font at Runtime

If the font data already resides in long-lived memory, register its descriptor at runtime without using the linker-section macro:

```c
#include <stdint.h>
#include "lvsf/lvsf_ft_reg.h"
#include "lvsf/lvsf_font_manager.h"
#include "app_font_data.h"

static const lv_font_freetype_lib_dsc_t app_memory_font_lib =
{
    .font_lib_size = APP_FONT_DATA_SIZE,
    .font_lib_data = (const char *)app_font_data,
    .font_lib_name = "MemoryFont",
};

static int register_memory_font(void)
{
    return lvsf_font_register_lib(&app_memory_font_lib, "MemoryFont", 10);
}
```

`lvsf_font_register_lib()` registers the entry without immediately validating the font data. The manager opens the font and creates the requested managed font object when a lookup first selects the entry.

The manager neither takes ownership of nor fully copies the descriptor and font data; it retains the original pointers. Do not allocate the descriptor or its data on a function's stack. Do not release them immediately after clearing the managed font cache either: the font entry still exists, and a later lookup may access those pointers again. There is currently no public API for unregistering this type of entry by name.

Use a unique, unregistered value for `name`. The current API reuses an existing entry if either its descriptor pointer or its name matches. If the name collides with a runtime file entry, the existing entry's source attributes may be retained and a later lookup may still treat the source as a file path.

During registration, the manager reinserts the current entry according to `priority`, before the first entry in the current list with a greater numeric value. This operation does not globally sort the list. Passing 0 also makes the entry the current default. Use a nonzero priority if the registration must not change the default font.

When `font_lib_size == 0`, `lvsf_font_register_lib()` can also store a file path, but `lvsf_font_unload_ex()` still cannot unregister the resulting entry. Use `lvsf_font_load_ex()` or the lazy registration API described above for ordinary runtime font files.

## Getting and Using Fonts

### Looking Up a Font by Name

Use `lvsf_font_get()` when the application must select a specific registered font:

```c
static int set_label_font(lv_obj_t *label)
{
    lv_font_t *font = lvsf_font_get("AppFont", 24);

    if (font == NULL)
    {
        return -1;
    }

    lv_obj_set_style_text_font(label, font,
                               LV_PART_MAIN | LV_STATE_DEFAULT);
    return 0;
}
```

The function returns `NULL` if the font does not exist, the font source cannot be opened, the format is unsupported, or the managed font object cannot be created. An explicit lookup by name does not check whether the entry is enabled.

When `size` is 0, the manager uses `FONT_NORMAL`. Pass an explicit pixel size in application code to make styling and memory requirements clear.

### Selecting a Font by Size

`lvsf_get_font_from_size(size)` does not name a font. The manager traverses enabled entries in their current order and returns the first font for which it can create the requested size:

```c
static lv_font_t *get_font_for_size(uint16_t size)
{
    return lvsf_get_font_from_size(size);
}
```

If every enabled font fails and the current default entry is disabled, the manager tries that default entry once more. Switch to another default before disabling the current one.

The default entry, entry order, and enabled state are independent:

- `lvsf_font_set_default(name)` changes the entry used by `lvsf_font_get("default", size)`, but does not enable the font or move it to the beginning of the entry list.
- `lvsf_font_set_enable(name, 0)` affects only the enabled-entry traversal phase of automatic selection and does not prevent `lvsf_font_get(name, size)` from returning the font.
- `lvsf_font_set_priority(name, priority)` changes the named entry's priority and reinserts that entry in the current list; it does not globally sort the list.
- `lvsf_font_set_order(names, count)` moves the named fonts to the beginning of the entry list in array order.

```c
static void select_app_fonts(void)
{
    char *font_order[] = {"AppFont", "BackupFont"};

    lvsf_font_set_enable("AppFont", 1);
    lvsf_font_set_order(font_order, 2);
}
```

Use `"default"` only as the special lookup name passed to `lvsf_font_get()`. Other management APIs handle this name inconsistently. Pass the font's actual registered name when changing the default, enabled state, or priority, or when clearing its cache.

`LV_EXT_FONT_GET()` accepts the size categories from `LVSF_FONT_SMALL` through `LVSF_FONT_SUPER`. Values from 0 through 6 are converted to their corresponding predefined pixel sizes; they are not treated as pixel sizes from 0 through 6. Use `lvsf_get_font_from_size()` or `lvsf_font_get()` for an arbitrary pixel size.

### Setting a Fallback Font

When the current font does not contain a glyph, LVGL follows its `fallback` pointer:

```c
static int set_app_font_fallback(void)
{
    lv_font_t *primary = lvsf_font_get("AppFont", 24);
    lv_font_t *fallback = lvsf_font_get("BackupFont", 24);

    if (primary == NULL || fallback == NULL || primary == fallback ||
            fallback->fallback != NULL)
    {
        return -1;
    }

    primary->fallback = fallback;
    return 0;
}
```

The fallback chain must not contain a cycle. Checking only `primary != fallback` does not rule out an indirect cycle; before assigning the pointer, verify that the target's existing fallback chain does not lead back to `primary`. The simplified code above accepts only a target that has no fallback of its own. `fallback` is not reference-counted, so the target font must outlive every font that refers to it. Replace all fallback references before unloading the target font or clearing its cache.

When the font manager releases a managed font, it clears `fallback` pointers to that font in other managed fonts. Fonts created directly by the application are outside this scan; the application must clear their fallback references.

If no font in the fallback chain contains the requested glyph, `LV_USE_FONT_PLACEHOLDER` determines whether LVGL draws a placeholder box. When the option is disabled, the missing character has zero advance width.

## Font Objects, Caches, and Memory

### Object Lifetime

A managed `lv_font_t` pointer remains valid only while its corresponding managed font object is cached. The following operations can invalidate it:

- `lvsf_font_clear_cache()` or `lvsf_font_clear_all_cache()` successfully releases the managed font object.
- `lvsf_font_unload_ex()` successfully unloads the runtime file font.
- The font manager or FreeType engine is shut down completely.

Do not retain font pointers indefinitely without tracking their source and unload timing. Widgets, shared styles, themes, fallback chains, and application-owned draw descriptors can all retain font pointers.

### Cache Layers

| Cache | Behavior | Configuration or cleanup |
| --- | --- | --- |
| Manager font cache | Retains one `lv_font_t` for each combination of font entry and pixel size, with no independent object-count limit | Replace references, then call `lvsf_font_clear_cache()` or unload the file-based font |
| FTC face and size caches | Multiple sizes from one font source share a face; configuration limits the number of face and size objects | `LV_FREETYPE_CACHE_FT_FACES`, `LV_FREETYPE_CACHE_FT_SIZES` |
| FTC glyph and charmap caches | Evicts entries based on total capacity and recreates them when needed | Uses the SDK cache capacity; applications normally do not manage these caches directly |

Continuously requesting new pixel sizes increases the number of managed font objects retained by the manager. Increasing the FTC size-object capacity does not limit this growth.

`lvsf_font_clear_cache(name)` attempts to clear all managed font objects for one entry, while `lvsf_font_clear_all_cache()` attempts to clear every entry. When reference checking is enabled, objects still referenced by display objects or themes are retained. Neither function returns a status, so an application cannot use the call to confirm that the cache is empty.

### Related Configuration

| Option | Default | Effect |
| --- | --- | --- |
| `FREETYPE_TINY_FONT` / `FREETYPE_NORMAL_FONT` | Tiny | Tiny uses the prebuilt FreeType library; Normal builds FreeType from the source in the repository |
| `LV_FREETYPE_CACHE_FT_FACES` | 0 | Number of face objects retained by FTC; 0 uses the FreeType default of 2 |
| `LV_FREETYPE_CACHE_FT_SIZES` | 0 | Number of size objects retained by FTC; 0 uses the FreeType default of 4 |
| `LVSF_FONT_MIN_FREE_HEAP` | 0 | Minimum number of system heap bytes that must remain after creating a runtime file font object; 0 disables the check |
| `LVSF_FONT_UNLOAD_REF_CHECK` | y | Checks display objects and themes for font references before releasing a managed font object |

`LVSF_FONT_MIN_FREE_HEAP` applies only to runtime file entries with `external=1`, and only when FreeType uses the system heap. The check does not apply to linker-section entries, entries registered with `lvsf_font_register_lib()`, or configurations that use a dedicated SRAM or PSRAM memory pool for FreeType.

The free-heap check runs after the font is opened and rejects a managed font object that would reduce the system heap below the configured threshold. Some font-manager metadata allocations still use assertions, so this option is not a general out-of-memory recovery mechanism.

## Switching and Unloading Runtime Font Files

`lvsf_font_unload_ex()` processes only runtime file entries. It cannot unregister linker-section fonts or entries registered with `lvsf_font_register_lib()`.

Unload a runtime file font in the LVGL thread in the following order:

1. Stop every code path that looks up the font by name.
2. If the font is the current default entry, call `lvsf_font_set_default()` first to select another registered and usable font.
3. Call `lvsf_font_set_enable(name, 0)` to exclude the font from the enabled-entry traversal phase of automatic selection.
4. Replace every font reference held by widgets, shared styles, themes, fallback chains, and application data.
5. If an object is deleted with `lv_obj_del_async()`, wait until the object has actually been deleted.
6. Call `lvsf_font_unload_ex()` with the full font path.
7. After unloading succeeds, do not access the old font pointers.

The code below handles only one widget and assumes that the caller has already changed the default entry and cleared every other reference:

```c
static int unload_app_font(lv_obj_t *label)
{
    if (lvsf_font_set_enable(APP_FONT_NAME, 0) != 0)
    {
        return -1;
    }

    lv_obj_set_style_text_font(label, LV_FONT_DEFAULT,
                               LV_PART_MAIN | LV_STATE_DEFAULT);

    return lvsf_font_unload_ex(APP_FONT_PATH);
}
```

`lvsf_font_unload_ex()` returns -1 if at least one matching managed font object remains referenced and is therefore retained. A return value of 0 means that no object was retained because of a reference, but it can also mean that no entry matched.

The function also accepts a registered font name, a directory prefix, or `NULL`; `NULL` matches every runtime file entry. Pass the full path to avoid unintentionally broadening the unload scope.

`LVSF_FONT_UNLOAD_REF_CHECK` is enabled by default. It checks references in display objects and themes, but cannot cover styles that are not attached to widgets, the per-span styles owned by `lv_spangroup` spans, draw descriptors retained by the application, or `fallback` pointers in fonts outside the manager. Reference checking is an additional pre-release check; it does not replace explicit reference cleanup by the application.

```{warning}
Do not use `lvsf_font_manager_deinit()` or `lvsf_font_unload()` to unload a single font. The current global cleanup path preserves only the first built-in entry, may remove other linker-section fonts, and does not rescan those fonts while the first built-in entry remains present. Applications should normally leave startup and shutdown of the entire font engine to the SDK lifecycle.
```

## Troubleshooting

| Symptom | Checks |
| --- | --- |
| `lvsf_font_load_ex()` returns -1 | Verify that the file system is mounted, the path is readable, the font uses TrueType outlines, the name is unique, the size array is valid, and the system heap meets the configured threshold |
| A batch load returns 1, but a requested size cannot be retrieved | A return value of 1 means only that at least one size succeeded; call `lvsf_font_get()` for each required size |
| `lvsf_font_get()` returns `NULL` | Verify the registered name and its case, the lifetime of the descriptor and data, access to the font source, and available memory |
| A character appears as a box or is missing | Neither the current font nor its fallback chain contains the glyph; also check `LV_USE_FONT_PLACEHOLDER` |
| Some characters differ in size or style | The fallback font renders the missing glyphs, and its metrics or glyph style differ from the primary font |
| Heap usage increases as new sizes are requested | The manager retains a font object for every new size; replace references and then clear the managed font cache |
| Automatic selection returns an unexpected font | Check the entry order, enabled state, and default entry together; changing the default does not change the order |
| `lvsf_font_unload_ex()` returns -1 | A display object, theme, or style still references the font; replace every reference by following the unloading sequence above, then try again |

## Common API Functions

Font manager functions are declared in _lvsf_font.h_ and _lvsf_font_manager.h_; the linker-section registration macro is declared in _lvsf_ft_reg.h_. These interfaces are available only when `LV_USING_FREETYPE_ENGINE` is enabled.

| API | Purpose | Key semantics |
| --- | --- | --- |
| `lvsf_font_manager_init(cache_size)` | Initializes the manager in a custom startup sequence | Returns 0 on success; normally unnecessary during standard SDK startup |
| `lvsf_font_load_ex(path, sizes)` | Registers a DFS file and immediately creates requested sizes | Returns 1 if at least one size succeeds; returns -1 on path, registration, or all-size failure |
| `LVSF_FREETYPE_FONT_REGISTER(name)` | Registers a font descriptor in the linker section | Requires `name_lib` to have external linkage |
| `lvsf_font_register_lib(lib, name, priority)` | Registers a font library descriptor at runtime | Validation is deferred; the descriptor and data must remain valid |
| `lvsf_font_register(config)` | Registers or configures a font entry for lazy loading | For a new path, `external=1` creates a runtime file entry |
| `lvsf_font_register_batch(configs, count)` | Registers multiple configurations in order | Does not roll back earlier successful registrations on failure |
| `lvsf_font_get(name, size)` | Retrieves a font by name | Returns a borrowed pointer; does not check the enabled state |
| `lvsf_get_font_from_size(size)` | Selects a font automatically in the current order | Tries enabled entries first |
| `lvsf_font_set_enable(name, enable)` | Controls whether the enabled-entry traversal phase includes the font | Does not prevent named lookup; a disabled default entry may still be retried |
| `lvsf_font_set_default(name)` | Sets the current default entry | Does not change the enabled state or entry order |
| `lvsf_font_set_priority(name, priority)` | Changes and reinserts one named font | Does not globally sort the entry list |
| `lvsf_font_set_order(names, count)` | Moves the named fonts to the beginning of the entry list | Preserves array order; no return value |
| `lvsf_font_unload_ex(path)` | Unloads matching runtime file fonts | Pass the full path; may return 0 when nothing matches |
| `lvsf_font_clear_cache(name)` | Attempts to clear all managed font objects for one entry | No return value; referenced objects may be retained |
| `lvsf_font_clear_all_cache()` | Attempts to clear all managed font objects | Replace every external reference before calling |

Applications should use the font manager instead of calling the low-level `lv_freetype_*` APIs directly. Low-level calls do not synchronize the font entries, size keys, or object lifetimes tracked by the font manager.

## References

- Font manager implementation: _middleware/lvgl/lvsf/lvsf_font_manager.c_
- Font adapter implementation: _middleware/lvgl/lvsf/lv_freetype.c_
- [LVGL v8 font documentation](https://docs.lvgl.io/8.3/overview/font.html)
- [FreeType reference documentation](https://freetype.org/freetype2/docs/reference/ft2-index.html)
