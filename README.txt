Command For Renaming Files:
    find <new directory> -type f -print0 | xargs -0 sed -i 's/<old name>/<new name>/g'
Command For Renaming Directories:

    find <new directory> -name "*<old directory>*" -exec rename 's/<old directory>/<new directory>/g' {} +
