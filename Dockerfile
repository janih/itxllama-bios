FROM balenalib/i386-ubuntu:xenial

ENV DEBIAN_FRONTEND noninteractive
ENV TZ=UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install the exact packages specified in README.md
RUN apt update -y && \
    apt install git build-essential m4 bison flex python texinfo gnat -y && \
    apt clean && rm -rf /var/lib/apt/lists/*

# Set up working directory
WORKDIR /workspace

# Copy source code
COPY . /workspace

# Make build scripts executable
RUN chmod +x build-xgcc.sh

# Default to bash shell for interactive use
ENTRYPOINT ["/bin/bash"]
