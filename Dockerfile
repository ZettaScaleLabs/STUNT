FROM ubuntu:jammy as build

# first let's build the runtime, the python binding and the python api
ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install git python3-venv python3-pip equivs musl-tools build-essential devscripts debhelper pkg-config dpkg-dev equivs libpython3-dev python-is-python3 python3-distutils -y

# install rust
RUN curl --proto "=https" --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain stable -y


# clone zenoh-flow runtime and zenoh-flow-python repos
RUN git clone https://github.com/gabrik/zenoh-flow-runtime /root/zenoh-flow-runtime
RUN git clone https://github.com/eclipse-zenoh/zenoh-flow-python /root/zenoh-flow-python -b feat-typed-inputs-outputs

# build zenoh-flow-runtime
RUN bash -c "source /root/.cargo/env && cd /root/zenoh-flow-runtime && cargo build --release --all-targets"

# build zenoh-flow-python
RUN bash -c "source /root/.cargo/env && cd /root/zenoh-flow-python && cargo build --release --all-targets"

# build zenoh-flow-python apis
RUN bash -c "source /root/.cargo/env && cd /root/zenoh-flow-python && python3 -m venv venv && source venv/bin/activate && cd zenoh-flow-python && pip3 install -r requirements-dev.txt && maturin build --release"

RUN git clone https://github.com/ZettaScaleLabs/STUNT
RUN cd STUNT/stunt-python && python3 setup.py bdist_wheel

FROM ubuntu:jammy as stunt
LABEL authors="Gabriele Baldoni"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install python3-pip -y

# Where configuration will reside
RUN mkdir -p /etc/zenoh-flow

# where python wrapper will reside
RUN mkdir -p /var/zenoh-flow/python/

# VOLUME ["/var/zenoh-flow/flows"]
# VOLUME ["/var/zenoh-flow/nodes"]

# copy zenoh-flow runtime zenoh configuration and wrappers configuration
COPY etc/zenoh-runtime.json /etc/zenoh-flow/zenoh-runtime.json
COPY etc/py-loader.yaml /etc/zenoh-flow/py-loader.yaml
COPY nodes/ /var/zenoh-flow/nodes
COPY var /var/zenoh-flow/var
COPY graphs/drive /var/zenoh-flow/flows

# copy the runtime into bin
COPY --from=build /root/zenoh-flow-runtime/target/release/runtime /usr/local/bin

# copy python wrappers
COPY --from=build /root/zenoh-flow-python/target/release/lib*.so /var/zenoh-flow/python/

# copy python api
COPY --from=build /root/zenoh-flow-python/target/wheels/*.whl /root/
COPY --from=build /STUNT/stunt-python/dist/*.whl /root/

RUN bash -c "pip3 install /root/*.whl"

RUN bash -c "rm /root/*.whl"

# COPY start.sh /usr/bin/start.sh
# RUN chmod +x /usr/bin/start.sh

# clean up
# RUN  bash -c "source /root/.cargo/env && rustup toolchain remove stable && rm -rf /root/.cargo"
# RUN  bash -c "rm -rf /root/zenoh-flow-python/venv"
RUN  DEBIAN_FRONTEND=noninteractive apt clean && apt autoclean

CMD sleep infinity
#"/usr/bin/start.sh"