# FicTrac makefile
# All leading spaces must be tabs!!

# Build directories
BUILDDIR := build
TARGETDIR := bin
EXECDIR := exec
SRCDIR := src
LIBDIR := lib
INCLDIR := include

# Input
CC := g++
# CC := clang --analyze # and comment out the linker last line for sanity
# TARGET := $(TARGETDIR)/fictrac
TARGET_LIB := $(LIBDIR)/libfictrac.a
SOURCES := $(shell find $(SRCDIR) -type f -name *.cpp)
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/src/%,$(SOURCES:.cpp=.o))

# Build flags
CFLAGS := -O3 -Wall -c -fmessage-length=0 -std=c++11 -Wno-unused-function -march=native -DBOOST_LOG_DYN_LINK
INCL := -I $(INCLDIR)
LDFLAGS := -L $(LIBDIR)
LDLIBS := -lfictrac -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lboost_log -lboost_log_setup -lboost_filesystem -lboost_system -lboost_thread -lpthread -lnlopt

# $(TARGET): $(OBJECTS)
  # $(CC) $^ -o $(TARGET) $(LIB)"; $(CC) $^ -o $(TARGET) $(LIB)

# Ensure build directories are present
pre_build:
	mkdir -p $(BUILDDIR) $(BUILDDIR)/src $(BUILDDIR)/exec $(TARGETDIR) $(LIBDIR)

# Compile the core library
$(TARGET_LIB): $(OBJECTS)
	ar rcs $(TARGET_LIB) $(OBJECTS)

# Build the core library modules
$(BUILDDIR)/src/%.o: $(SRCDIR)/%.cpp
	$(CC) $(CFLAGS) $(INCL) -o $@ $<

# Build the binaries
$(BUILDDIR)/exec/%.o: $(EXECDIR)/%.cpp
	$(CC) $(CFLAGS) $(INCL) -o $@ $<

# Executables
configGui: pre_build $(TARGET_LIB) $(BUILDDIR)/exec/configGui.o
	$(CC) -o $(TARGETDIR)/$@ $(BUILDDIR)/exec/$@.o $(LDFLAGS) $(LDLIBS)

# Debugging
print-%:
	@echo $* = $($*)

clean:
	rm -rf $(BUILDDIR) $(TARGETDIR) $(LIBDIR)

.PHONY: clean
