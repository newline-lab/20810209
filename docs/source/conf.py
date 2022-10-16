# Configuration file for the Sphinx documentation builder.

# -- Project information

project = '20810209-CRSL'
copyright = '2022, UNIROMA3'
author = 'IGEA - UNIROMA3'

release = '0.1'
version = '0.1.0'

# -- General configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'sphinx_copybutton',
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

templates_path = ['_templates']

copybutton_prompt_text = "$ "

# -- Options for HTML output

html_theme = 'sphinx_rtd_theme'

# -- Options for EPUB output
