# Github basics {#git-basics status=ready}

## Github terms explained

### Repository

A repo (short for repository), or Git project, encompasses the entire collection of files and folders associated with a project, along with each file’s revision history.

The main Duckietown repository is https://github.com/duckietown/Software/

### Branch

A branch is a version of the main code, that you can work on and it's changes won't affect the main code until it is merged into the master branch. When several people collaborate on a project, it makes sense for each developer to work on his own branch.

### Fork

A fork is basically a copy of someone else's repository. Usually, you cannot create branches or change code in other people's repos, thats why you create your own copy of it. This is called `forking`.

## Github basic commands

### Fork a repository

To fork (creating a copy of a repository, that does not belong to you), you simply have to go to the repository's webpage dashboard and click fork on the upper right corner.

### Clone a repository

To clone a repository, either copy the HTTPS or SSH link given on the repository's webpage. Then invoke following command to download the git repository onto the local computer (actual directory you are in right now).

    $ git clone git@github.com:USERNAME/REPOSITORY.git
    
If you have SSH setup properly, you can directly download it. If you are using the HTTPS then github will ask for your credentials.

### Create a new branch

After you successfully cloned a repository, you may want to work on your own branch in order not to cause any chaos in the master branch. It is usually protected against changes. For this, you can branch out from the master or any other branches by invoking the command

    $ git checkout -b ![branch-name]
    
To see which branch you are working on you can either use both of these commands

    $ git branch
    $ git status
    
The latter provides more information on which files you might have changed, which are staged for a new commit or that you are up-to-date (everything is ok).

### Commit and Push changes

After you edited some files, you want to push your changes from the local to the remote location. In order to do so, first do a double-check on which files you have changed and if things look legitimate. Invoke

    $ git status
    
and check the output. There will be several files, that show up in red. These are files you have changed, but not yet added for a future commit. Most of the time you want to push all your changes so you add them to your commit by executing

    $ git add --all
    
If you do not want to add all files, single files can be added. Then you need to specify each single file

    $ git add ![file-path]
    
After you solved this, add a commit message to let collaborators know, what you have changed:

    $ git commit -m "![commit-message]"
    
If everything went smooth without any issues you are ready to push your changes to your branch:

    $ git push origin ![branch-name]

### Fetch new branches

If new branches have been pushed recently to the repository and you don't have them you can invoke a

    $ git fetch --all
    
to see all new branches and checkout to those.

### Delete branches

To delete a local branch execute (you cannot be on the branch that you are going to delete!):

    $ git branch -d ![branch-name]

To delete a remote branch you need to push the delete command:

    $ git push origin --delete ![branch-name]
    
### Open a pull request

If you are working on another branch than the master or if you forked a repository and want to propose changes you made into the master, you can open a so-called `pull-request`. In order to do so, press the corresponding tab in the dashboard of a repository and then press the green button `New pull request`. You will be asked which branch from which fork you want to merge.
    
### Submitting issues

If you are experiencing issues with any code or content of a repository (such as this operating manual you are reading right now), you can submit issues. For doing so go to the dashboard of the corresponding repository and press the `Issues` tab where you can open a new request. 

For example you encounter a bug or a mistake in this operating manual, please visit this [repository](https://github.com/duckietown/docs-opmanual_duckiebot/issues) to open a new issue.
