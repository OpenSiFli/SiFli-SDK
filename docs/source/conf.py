# -*- coding: utf-8 -*-
# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
import datetime

current_year = datetime.datetime.now().year
project = 'SiFli SDK编程指南'
copyright = '2019 - {} 思澈科技（上海）有限公司'.format(current_year)
author = 'SiFli'
language = 'zh_CN'
version = os.environ.get('SIFLI_DOC_VERSION', 'latest')

if "SF32LB55X" in tags:
    chip = 'sf32lb55x'
elif "SF32LB56X" in tags:
    chip = 'sf32lb56x'
elif "SF32LB58X" in tags:
    chip = 'sf32lb58x'  
elif "SF32LB52X" in tags:
    chip = 'sf32lb52x'    
else:
    chip = 'sf32lb52x'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["breathe", 
              "myst_parser", 
              "sphinx_rtd_theme",
              # "sphinx_book_theme",
              "sphinx_multiversion",
              "sphinxcontrib.mermaid",
              "sphinx_copybutton",
              "sphinx.ext.intersphinx",
              #"sphinx_ext_substitution",
              "sphinx_design"
              ]

templates_path = ['_templates']
exclude_patterns = []

numfig = False


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# html_theme = 'sphinx_rtd_theme'
# html_theme = 'sphinx_book_theme'
html_theme = 'shibuya'
html_static_path = ['_static']

html_context = {
    "versions": [
        ("latest", "latest"),
        # 只保留latest版本，其他版本通过JavaScript动态加载
    ],
    "chips": [
        ("SF32LB52x", "sf32lb52x"),
        ("SF32LB56x", "sf32lb56x"),
        ("SF32LB58x", "sf32lb58x"),
        ("SF32LB55x", "sf32lb55x"),
    ],
    "current_version": version,
    "current_chip": chip,

}

html_logo = './_static/logo_white.png'
html_favicon = './_static/logo_favicon.png'

html_show_sourcelink = False
#html_show_copyright = False

html_theme_options = {
    'collapse_navigation' : False,
    'style_nav_header_background': 'white',
    'logo_only': False,
    "accent_color": "blue",
}

html_css_files = [
    'css/custom.css',
    'css/feedback.css',
]

html_js_files = [
    'js/baidu.js',
    'js/feedback.js',
]

# -- Options for Breathe ----------------------------------------------------

breathe_projects = {"coresdk": os.path.join(os.path.dirname(__file__), r"../doxygen/xml")}
breathe_default_project = "coresdk"
breathe_domain_by_extension = {"h" : "c"}
breathe_implementation_filename_extensions = ['.c', '.cc', '.cpp']

# -- Options for myst_parser ----------------------------------------------------

myst_enable_extensions = [
    "amsmath",
    "attrs_inline",
    "attrs_block",
    "colon_fence",
    "deflist",
    "dollarmath",
    "fieldlist",
    "html_admonition",
    "html_image",
    "replacements",
    "smartquotes",
    "strikethrough",
    "substitution",
    "tasklist",
]

myst_fence_as_directive = ["mermaid"]

# temp solution to include files according to chip series, esp-docs has more elegant solution. It extends `toctree` directive by `toctree_filter.py`
# `only`和`toctree`指示符不能嵌套使用，当把toctree嵌在only里时，虽然页面上的目录树可以正常显示，但侧边导航栏的显示不正确，而且即使没有被toctree引用的文件也会被转成html
if "SF32LB55X" in tags:
    # HAL
    exclude_patterns = ["**/audcodec.md", "**/audprc.md", "**/atim.md", "**/facc.md", "**/fft.md", "**/hash.md", "**/mpi.md"]
    # App note
    exclude_patterns += ["**/quick_start_52x_56x.md"]
    # Middlware
    exclude_patterns += ["**/bt_service.md"]
    # Example 
    exclude_patterns += ["**/hash/**", "example/bt/**"]
    # app_development
    exclude_patterns += ["**/startup_flow_sf32lb52x.md"]


if "SF32LB58X" in tags:
    # HAL
    exclude_patterns = ["**/hash.md", "**/psram.md", "**/qspi.md"]
    # App note
    exclude_patterns += ["**/quick_start_55x.md", "**/memory_usage.md"]
    # Middlware
    exclude_patterns += []
    # Example 
    exclude_patterns += ["**/hash/**"]
    # app_development
    exclude_patterns += ["**/startup_flow_sf32lb52x.md"]


if "SF32LB56X" in tags:
    # HAL
    exclude_patterns = ["**/dsi.md", "**/psram.md", "**/qspi.md"]
    # App note
    exclude_patterns += ["**/quick_start_55x.md", "**/memory_usage.md"]
    # Middlware
    exclude_patterns += []
    # Example 
    exclude_patterns += []    
    # app_development
    exclude_patterns += ["**/startup_flow_sf32lb52x.md"]


if "SF32LB52X" in tags:
    # HAL
    exclude_patterns = ["**/busmon.md", "**/dsi.md", "**/facc.md", "**/fft.md", "**/nnacc.md", "**/psram.md", "**/qspi.md"]
    # App note
    exclude_patterns += ["**/quick_start_55x.md", "**/memory_usage.md"]
    # Middlware
    exclude_patterns += []
    # Example 
    exclude_patterns += ["example/multicore/**"]


exclude_patterns += ["example/52x_mode_test/*", "example/gpadc/*", "example/micropython/*",
                     "example/hal_example/*", "**/get-started-keil.md"] 