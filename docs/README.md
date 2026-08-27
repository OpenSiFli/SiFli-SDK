# How to build the document

## Install toolchain
1. Install Python 3.x
1. Execute `pip install -r requirements.txt` in command line under the `docs` folder. It would install all modules needed by the document building

## Build the document
Run batch file `make_all.bat` in Windows command line. It would build documents for all SoC series. You can also execute command below to build only sf32lb52x document and the generated document locates in `build_52x` folder.
```shell
python generate_docs.py 52x
```

## Configure Chinese documentation AI Q&A

The `ALGOLIA_DOCSEARCH_VERSIONS` GitHub Actions variable holds the
version-specific Algolia configuration.  Configure one Agent ID per chip under
`agent_ids`; the documentation build selects the ID matching the page's chip.

```json
{
  "latest": {
    "app_id": "<Algolia application ID>",
    "search_api_key": "<DocSearch search-only API key>",
    "agent_ids": {
      "sf32lb52x": "<52x Agent ID>",
      "sf32lb55x": "<55x Agent ID>",
      "sf32lb56x": "<56x Agent ID>",
      "sf32lb57x": "<57x Agent ID>",
      "sf32lb58x": "<58x Agent ID>"
    }
  }
}
```

Each Agent must have search access to all five `sdk_<version>_sf32lb*` indices.
Its Agent Studio instructions determine the default chip when users do not name
one; explicit chip requests are resolved by searching the named chip index.

