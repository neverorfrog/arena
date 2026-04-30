FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y libstdc++6 && rm -rf /var/lib/apt/lists/*
ENTRYPOINT ["/workspace/scripts/docker_entrypoint_circus.sh"]
