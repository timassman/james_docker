#!/bin/bash

# Example usage:
# ./git_repos.sh status
# ./git_repos.sh pull

# store the current dir
CUR_DIR=$(pwd)

# Let the person running the script know what's going on.
echo -e "\nGit $1 for all repositories...\n"

# Find all git repositories and update it to the master latest revision
for i in $(find . -name ".git" | cut -c 3-); do
    echo "";
    echo "$i";

    # We have to go to the .git parent directory to call the pull command
    cd "$i";
    cd ..;

    # finally status or pull, depending on argument stored in $1
    git $1;

    # lets get back to the CUR_DIR
    cd $CUR_DIR
done

echo -e "\nComplete!\n"