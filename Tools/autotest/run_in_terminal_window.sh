#!/usr/bin/env bash

# Try to run a command in an appropriate type of terminal window
# depending on whats available
# Sigh: theres no common way of handling command line args :-(
name="$1"
shift
echo "RiTW: Starting $name : $*"

if [ -z "$SITL_RITW_MINIMIZE" ]; then
    SITL_RITW_MINIMIZE=1
fi

if [ -n "$SITL_RITW_TERMINAL" ]; then
  # create a small shell script containing the command to run; this
  # avoids problems where "screen" expects arguments in
  # argv[1],argv[2],argv[3] where gnome-terminal expects the command
  # to run be in argv[n+1] where argv[n] is "-e"
  # this should work with:
  # export SITL_RITW_TERMINAL="screen -D -m"
  # export SITL_RITW_TERMINAL="gnome-terminal -e"
  # export SITL_RITW_TERMINAL="konsole -e"

  test -z "$TMPDIR" && TMPDIR="/tmp/"
  FILENAME="ritw-`date '+%Y%m%d%H%M%S'`"
  FILEPATH="$TMPDIR/$FILENAME"
  echo "#!/bin/sh" >"$FILEPATH"
  printf "%q " "$@" >>"$FILEPATH"
  chmod +x "$FILEPATH"
  $SITL_RITW_TERMINAL "$FILEPATH" &
elif [ -n "$TMUX" ]; then
  tmux new-window -dn "$name" "$*"
elif [ "$(uname | grep -o CYGWIN)" == "CYGWIN" ]; then
    cmd.exe /c start "$cmd" "$@" > /dev/null 2>&1
    exit 0
fi
elif [ -n "$DISPLAY" -a -n "$(which xterm)" ]; then
  if [ $SITL_RITW_MINIMIZE -eq 1 ]; then
      ICONIC=-iconic
  fi
  xterm $ICONIC -xrm 'XTerm*selectToClipboard: true' -xrm 'XTerm*initialFont: 6' -n "$name" -name "$name" -T "$name" -hold -e $* &
elif [ "$(uname | grep -o CYGWIN)" == "CYGWIN" ]; then
    cmd.exe /c start "$cmd" "$@" > /dev/null 2>&1
    exit 0
fi
elif [ "$(uname | grep -o CYGWIN)" == "CYGWIN" ]; then
    cmd.exe /c start "$cmd" "$@" > /dev/null 2>&1
    exit 0
fi
elif [ -n "$STY" ]; then
  # We are running inside of screen, try to start it there
  screen -X screen -t "$name" bash -c "cd $PWD; $*"
elif [ -n "$ZELLIJ" ]; then
  # Create a new pane to run
  zellij run -n "$name" -- "$1" "${@:2}"
else
  filename="/tmp/$name.log"
  echo "RiTW: Window access not found, logging to $filename"
  cmd="$1"
  shift
# the following "true" is to avoid bash optimising the following call
# to avoid creating a subshell.  We need that subshell, or
# _fdm_input_step sees ArduPilot has no parent and kills ArduPilot!
  ( : ; "$cmd" $* &>"$filename" < /dev/null ) &
fi
exit 0
