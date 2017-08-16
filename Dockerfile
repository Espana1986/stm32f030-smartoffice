FROM debian:8

MAINTAINER Danielle Jackson

RUN apt-get update && apt-get install -y gcc-arm-none-eabi gdb-arm-none-eabi build-essential