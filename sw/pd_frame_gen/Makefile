
override CFLAGS += -Wall -O3

sources := $(wildcard src/*.c)
objects := $(sources:src/%.c=obj/%.o)

all: output_dir pd_frame_gen

pd_frame_gen:  $(objects) 
	${CC} -o $@    $^ $(LDFLAGS) -lpthread

$(objects): obj/%.o: src/%.c
	${CC} -o $@ $^ -c $(CFLAGS)

output_dir:
	@mkdir -p obj

clean:
	rm -Rf  *.o  .*.o  .*.o.* pd_frame_gen obj
