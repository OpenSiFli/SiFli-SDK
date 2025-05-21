FROM ubuntu:22.04 AS core

# Install all dependencies
RUN apt-get update && apt-get install -y \
    sudo git wget flex bison gperf \
    python3 python3-pip python3-venv \
    cmake ninja-build ccache \
    libffi-dev libssl-dev \
    dfu-util libusb-1.0-0 \
    && apt-get clean

# Create a sifli user with sudo
ARG USERNAME=sifli
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --gid $USER_GID --uid $USER_UID --shell /bin/bash --create-home $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~

FROM core AS toolchain

# Install the toolchain
COPY install.sh /home/sifli/src/install.sh
COPY tools /home/sifli/src/tools
COPY version.txt /home/sifli/src/version.txt

RUN sudo --user=sifli /home/sifli/src/install.sh

# ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~ ~~~~~~~~~~

FROM core AS devcontainer

# Install the toolchain
COPY --from=toolchain /home/sifli/.sifli /home/sifli/.sifli

COPY .devcontainer/motd /etc/motd
RUN chmod +x /etc/motd && \
    echo "alias init='source /home/sifli/src/export.sh'" >> /home/sifli/.bashrc && \
    echo '[ ! -z "$TERM" -a -r /etc/motd ] && /etc/motd' >> /home/sifli/.bashrc && \
    echo '[ -f /home/sifli/src/export.sh ] && source /home/sifli/src/export.sh' >> /home/sifli/.bashrc && \
    touch /home/sifli/.sudo_as_admin_successful

USER sifli
WORKDIR /home/sifli/src
ENV TERM=xterm-256color