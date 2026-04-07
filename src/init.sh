#!/bin/sh

export HOME=/root

echo -e "Welcome to \e[96m\e[1mStarry OS\e[0m!"
env
echo

echo -e "Use \e[1m\e[3mapk\e[0m to install packages."
echo

# Do your initialization here!

cd ~
# /bin/test-roboctl
# /tennis run /data
# /tennis run /data/tennis_cv181x_bf16.cvimodel
/tennis follow /data/tennis_cv181x_bf16.cvimodel
# sh --login
