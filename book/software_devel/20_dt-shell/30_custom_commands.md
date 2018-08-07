# Custom commands {#dt-shell-custom-commands status=draft}

Maintainer: Andrea Daniele

`dt-shell` downloads all the commands from the git repository
[duckietown/duckietown-shell-commands](https://github.com/duckietown/duckietown-shell-commands).
If you want to contribute to the shell and create new commands, please follow the instructions in the section
[](#dt-shell-create-custom-command).



## Downloading the commands {#dt-shell-pull-commands-repo}

You can download the source code of the commands by forking the `duckietown-shell-commands`
repository and pulling your fork locally. The repository can be found at this
[link](https://github.com/duckietown/duckietown-shell-commands).



## Understanding the structure of the repository {#dt-shell-understand-repo}

Move to the directory where you pulled the commands repository. Run

    $ tree ./

You should be able to see a hierarchy of Python files that looks like the following,

```
./
├── README.md
├── __init__.py
├── lib
│   └── ...
│       ...
├── install
│   ├── __init__.py
│   ├── command.py
│   └── installed.flag
├── uninstall
│   ├── __init__.py
│   ├── command.py
│   └── installed.flag
...
├── exit
│   ├── __init__.py
│   ├── command.py
│   └── installed.flag
└── version
    ├── __init__.py
    ├── command.py
    └── installed.flag
```

Let us focus on the directories first. The directory `lib` is reserved to third-party libraries needed by the
commands. Each directory (except for `lib`) defines a command. You will recognize some of the core commands
(e.g., install, uninstall, version). The name of the directory defines the name of the command.

NOTE: A valid command name can contain only alphanumeric characters plus `_` and `-`.

Let us focus on files now.
`dt-shell` determines whether a command `![mycommand]` is **installed** by looking for the file
`./![mycommand]/installed.flag`.

All the command directories contain a default `__init__.py` file. Such file is the same for all the commands.
A standard template file `__init__.py.template` that you can copy if you wish to implement your own command
is available in the main level of the repository.

All the magic happens in the file `command.py`, where the logic of the command is implemented.
If you want to learn more about how to create your own command, read the section [](#dt-shell-create-custom-command).

This is all you need to know about the structure of the repository.



## Create a new command {#dt-shell-create-custom-command}

Remark: Before you can create a new command, you have to pull the source code locally as described in
[](#dt-shell-pull-commands-repo). It is also important that you are familiar with the structure of the
repository (described in [](#dt-shell-understand-repo)).


You can create a command `![mycommand]` by creating a directory in the main level of the repository with
the following structure

```
./
...
└── mycommand
    ├── __init__.py
    └── command.py
```

You can copy the files `__init__.py.template` and `command.py.template` that you find on the main level of
the repo into your new command directory renaming them as `__init__.py` and `command.py` respectively.

This is enough to get your new command in `dt-shell`. Launch `dt-shell` and run

    $ dt> mycommand

You should be able to see something like the following

```
dt> mycommand
You called the "mycommand" command, level 0, with arguments []
```

You can pass arguments to your command. For example, by running

    $ dt> mycommand --arg1 value1

the shell will return

```
dt> mycommand
You called the "mycommand" command, level 0, with arguments ['--arg1', 'value1']
```

When the user types in the command `mycommand` and presses <kbd>Enter</kbd>, the file `./![mycommand]/command.py`
will be used to serve the request.
A valid file `command.py` will have the following structure

```
from __future__ import print_function
from dt_shell import DTCommandAbs

class DTCommand(DTCommandAbs):

    help = 'Brief description of the command'     # please redefine this help message
    # name = <read-only> a string with the name of the command
    # level = <read-only> an integer indicating the level of this command. Follows the directory hierarchy
    # commands = <read-only> a dictionary of subcommands

    @staticmethod
    def command(shell, args):
        # this function will be invoked when the user presses the [Return] key and submits the command
        #
        #   shell   is the instance of DTShell hosting this command
        #   args    is a list of arguments passed to the command
        #
        # PUT YOUR CODE HERE
        print(
            'You called the "%s" command, level %d, with arguments %r' % (
                DTCommand.name,
                DTCommand.level,
                args
            )
        )


    @staticmethod
    def complete(shell, word, line):
        # this function will be invoked when the user presses the [Tab] key for auto completion.
        #
        #   shell   is the instance of DTShell hosting this command
        #   word    is the right-most word typed in the terminal (usually the string the user is trying to auto-complete)
        #   line    is the entire command
        #
        #   return  a list of strings. Each string is a suggestion for the user
        #
        # PUT YOUR CODE HERE
        return ['suggestion_1', 'suggestion_2']
```

You can find the same template in the file `command.py.template` in the main level of the repository.
You can recognize the default message printed above by the method `command(shell, args)` of the command `![mycommand]`.

The method `command(shell, args)` is invoked by the object `shell` when the user presses <kbd>Enter</kbd>
and submits the argument `args` to the command. The argument `shell` is the instance of DTShell hosting this
command, while `args` is the list of arguments passed to the command.

The method `complete(shell, word, line)` is invoked by the object `shell` when the user presses <kbd>Tab</kbd>
and requests auto-complete. This method should return a list of strings, with each string being a suggestion
for completing the command.


## Update an existing command {#dt-shell-edit-command}

Remark: Before you can create a new command, you have to pull the source code locally as described in
[](#dt-shell-pull-commands-repo). It is also important that you are familiar with the structure of the
repository (described in [](#dt-shell-understand-repo)).


You can follow the instructions in the section [](#dt-shell-create-custom-command) to learn how the shell
reacts to the input of the user and update your commands accordingly.



## Install third-party libraries

You can add third-party libraries to the `./lib/` directory using `git submodule` if you have access to a
public `git` repository that you can pull. Alternatively (but not suggested) you can simply copy your library
in the `./lib/` directory and push it together with the commands.

`dt-shell` prepends the `./lib/` path to the `$PYTHON_PATH` environment variable before calling your command function.
If you have your library in `./lib/my_lib/foo.py`, you can add the line `from my_lib import foo` at the very top
of your `command.py` file.

NOTE: `dt-shell` will take care of updating your library when the user runs `dt> update` if you use `git submodule`.
