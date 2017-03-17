CC = gcc
INCLUDES = -Iinclude
CFLAGS = -Wall -Wpedantic -std=c90
BINDIR = build
OBJDIR = build/obj

SOURCES = \
    src/vn/conv.c \
    src/vn/error.c \
    src/vn/error_detection.c \
    src/vn/sensors.c \
    src/vn/util.c \
    src/vn/math/matrix.c \
    src/vn/math/vector.c \
    src/vn/protocol/spi.c \
    src/vn/protocol/upack.c \
    src/vn/protocol/upackf.c \
    src/vn/sensors/compositedata.c \
    src/vn/sensors/ezasyncdata.c \
    src/vn/sensors/searcher.c \
    src/vn/xplat/criticalsection.c \
    src/vn/xplat/event.c \
    src/vn/xplat/serialport.c \
    src/vn/xplat/thread.c \
    src/vn/xplat/time.c
				
# Set the object file names, with the source directory stripped
# from the path, and the build path prepended in its place			
OBJECTS = $(SOURCES:src/%.c=$(OBJDIR)/%.o)

all: dirs lib

lib: libvnc.a

libvnc.a: $(OBJECTS)
	ar -cvq $(BINDIR)/libvnc.a $(OBJECTS)

# Create the directories used in the build
.PHONY: dirs
dirs:
	@mkdir -p $(BINDIR)
	@mkdir -p $(OBJDIR)
	@mkdir -p $(dir $(OBJECTS))

$(OBJDIR)/%.o: src/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(BINDIR)/libvnc.a
	rm -f $(OBJECTS)
