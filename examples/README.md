# Example FMU configurations

Note, this is a work in progress, so the contents of these configuration files
are expected to evolve over time as the feature set of the FMU expands and
matures.

I have added an additional feature on top of pure json that enables a top level
json file to include sub json files in specific locations of the tree (a bit
like a C preprocessor #include.)  Take a look at the "config.json" for more
detailed examples.

    "subsection": {
        "include": "file.json"
    }
