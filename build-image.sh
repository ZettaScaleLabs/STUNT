
#!/bin/bash
sg docker -c "docker build ./ -f ./Dockerfile -t gabrik91/stunt --no-cache" --oom-kill-disable