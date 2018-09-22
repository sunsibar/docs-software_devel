# Commands {#dt-shell-commands status=draft}

Maintainer: Andrea Daniele

`dt-shell` is modular, and the `pip` package does not contain commands. The first time you launch
the shell, the most recent version of the commands will be automatically downloaded and made available
to you. A barebones set of commands will be automatically installed, others will be available but not
installed.



## List commands

You can list all the commands available in dt-shell by running

    $ dt> commands [--core] [--installed] [--installable]


You should see something like the following

```
dt> commands
Core commands:
    commands
    install
    uninstall
    update
    version
    exit
    help

Installed commands:
    <empty>

Installable commands:
    aido18
    logs
```

Use the arguments `--core`, `--installed`, and `--installable` to filter the commands.



## Update list of commands

Run the command `dt> update` to download the most recent version of the commands available.

NOTE: `dt-shell` uses `git` to update the commands. If you get an error, please make sure that `git` is installed.



## Install a new command

Run `dt> commands --installable` to see the list of commands you can install.

Run `dt> install ![command_name]` to install a new command.



## Remove a command

Run `dt> commands --installed` to see the list of commands already installed.

Run `dt> uninstall ![command_name]` to remove a command.
