CONTAINER = stm32f030-build 
FLASHER = st-flash 

all:
	docker run --rm -v `pwd`:/host $(CONTAINER) /bin/bash -c "cd /host/firmware/; make"

clean:
	docker run --rm -v `pwd`:/host $(CONTAINER) /bin/bash -c "cd /host/firmware/; make clean"

flash: 	
	$(FLASHER) write ./firmware/main.bin 0x8000000 
	$(FLASHER) reset

erase:
	$(FLASHER) --erase-all -f nrf52

prepare:
	docker build -t $(CONTAINER) . 
