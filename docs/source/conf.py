# Configuration file for the Sphinx documentation builder.
import sphinx_copybutton
# -- Project information

project = '20810209-CRSL'
copyright = '2022, UNIROMA3'
author = 'IGEA - UNIROMA3'

release = '0.1'
version = '0.1.0'

# -- General configuration

extensions = [
    'sphinx_copybutton',
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}


html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    # General
    'display_version'           : False,
    'logo_only'                 : False,
    'prev_next_buttons_location': 'both',
    'style_external_links'      : True,
    # Navigation
    'collapse_navigation'        : False,
    'navigation_depth'           : -1,
    'sticky_navigation'          : True,
    'titles_only'                : False,
}

# Exclude Python and Bash prompts when copying code blocks
copybutton_prompt_text = r'>>> |\$ '
copybutton_prompt_is_regexp = True
