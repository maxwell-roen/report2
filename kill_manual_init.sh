#!/usr/bin/env bash
rosnode list | grep '/manual_initialization' | xargs rosnode kill
