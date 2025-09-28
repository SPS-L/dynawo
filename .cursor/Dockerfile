# Dockerfile for DYNAWO RTE Background Agents
# Author: Sustainable Power Systems Lab (SPSL) - https://sps-lab.org
# Contact: info@sps-lab.org

FROM ubuntu:22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set timezone
ENV TZ=Europe/London
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install system dependencies required for DYNAWO RTE development
# Following Ubuntu 22.04 package requirements
RUN apt-get update && apt-get install -y \
    git \
    gcc \
    g++ \
    gfortran \
    autoconf \
    pkgconf \
    automake \
    make \
    libtool \
    cmake \
    hwloc \
    openjdk-8-jdk \
    libblas-dev \
    liblpsolve55-dev \
    libarchive-dev \
    doxygen \
    doxygen-latex \
    liblapack-dev \
    libexpat1-dev \
    libsqlite3-dev \
    zlib1g-dev \
    gettext \
    patch \
    clang \
    python3-pip \
    libncurses5-dev \
    libreadline-dev \
    libdigest-perl-md5-perl \
    unzip \
    gcovr \
    lcov \
    libboost-all-dev \
    lsb-release \
    libxml2-utils \
    python3-lxml \
    python3-psutil \
    wget \
    libcurl4-openssl-dev \
    rsync \
    libopenblas-openmp-dev \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    # Additional useful packages for development
    python3 \
    python3-dev \
    curl \
    vim \
    htop \
    nano \
    graphviz \
    libzmq3-dev \
    libwebsocketpp-dev \
    bash-completion \
    xz-utils \
    sudo \
    # Required for Intel oneAPI installation
    ca-certificates \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# Install Intel oneAPI Advisor for performance analysis
RUN curl -fsSL https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
    | gpg --dearmor | tee /usr/share/keyrings/oneapi-archive-keyring.gpg >/dev/null 2>&1 && \
    echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" \
    | tee /etc/apt/sources.list.d/oneAPI.list >/dev/null && \
    apt-get update -y && \
    (apt-get install -y --no-install-recommends intel-oneapi-advisor || \
     apt-get install -y --no-install-recommends intel-hpckit) && \
    rm -rf /var/lib/apt/lists/*

# Build and install ZMQ++ from source (since libzmqpp-dev is not available in Ubuntu 22.04)
RUN cd /tmp && \
    git clone https://github.com/zeromq/zmqpp.git && \
    cd zmqpp && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd / && rm -rf /tmp/zmqpp

# Install Python packages
RUN pip3 install --no-cache-dir \
    lxml \
    psutil

# Set up environment variables for DYNAWO
ENV DYNAWO_LOCALE=en_GB
ENV DYNAWO_RESULTS_SHOW=true
ENV DYNAWO_BROWSER=firefox
ENV DYNAWO_NB_PROCESSORS_USED=2
ENV DYNAWO_BUILD_TYPE=Release
ENV DYNAWO_PYTHON_COMMAND=python3

# Set up Intel oneAPI environment
RUN if ! grep -q "oneapi/setvars.sh" /etc/profile; then \
      echo ". /opt/intel/oneapi/setvars.sh > /dev/null 2>&1 || true" >> /etc/profile; \
    fi

# Create a non-root user for development
RUN useradd -m -s /bin/bash ubuntu && \
    usermod -aG sudo ubuntu && \
    echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Set user and working directory as recommended for background agents
USER ubuntu
WORKDIR /home/ubuntu

# Set up Git configuration (can be overridden by user)
RUN git config --global user.name "SPSL Developer" && \
    git config --global user.email "info@sps-lab.org" && \
    git config --global init.defaultBranch main

# Create common directories
RUN mkdir -p /home/ubuntu/workspace /home/ubuntu/projects

# Set default shell to bash
SHELL ["/bin/bash", "-c"]

# Add helpful aliases and environment setup
RUN echo 'alias ll="ls -alF"' >> ~/.bashrc && \
    echo 'alias la="ls -A"' >> ~/.bashrc && \
    echo 'alias l="ls -CF"' >> ~/.bashrc && \
    echo 'export PATH=$PATH:/usr/local/bin' >> ~/.bashrc

# Expose common development ports (can be customized as needed)
EXPOSE 8080 8000 3000

# Default command
CMD ["/bin/bash"]
