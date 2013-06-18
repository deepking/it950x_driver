#!/bin/bash

wget -qO- https://raw.github.com/creationix/nvm/master/install.sh | sh
nvm install 0.10.10
nvm use 0.10.10
nvm alias default 0.10.10
npm install -g coffee-script
coffee -h
