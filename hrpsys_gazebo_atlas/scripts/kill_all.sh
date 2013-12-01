#!/bin/bash

# kill all rtm related program
ps -aux | grep rtm | awk '{print $2}' | xargs kill -KILL

# kill all ros related program
ps -aux | grep ros | awk '{print $2}' | xargs kill -KILL

ps -auxwww

