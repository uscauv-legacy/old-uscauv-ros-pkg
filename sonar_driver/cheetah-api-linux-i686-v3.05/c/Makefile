#==========================================================================
# (c) 2006-2008  Total Phase, Inc.
#--------------------------------------------------------------------------
# Project : Cheetah Sample Code
# File    : Makefile
#--------------------------------------------------------------------------
# Redistribution and use of this file in source and binary forms, with
# or without modification, are permitted.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#==========================================================================

#==========================================================================
# CONFIGURATION
#==========================================================================
OS=$(shell if [ -x /bin/uname ]; then /bin/uname; fi)

ifeq ($(OS),Linux)
SYSLIBS=-ldl
endif


#==========================================================================
# CONSTANTS
#==========================================================================
OUTDIR=_output
CC=gcc -m32
CFLAGS=-Wall -Werror
LD=$(CC)


#==========================================================================
# TARGETS
#==========================================================================
TARGETS=detect eeprom flash blast timing async

BINARIES=$(TARGETS:%=$(OUTDIR)/%)

BINARIES=$(TARGETS:%=$(OUTDIR)/%)

all : $(OUTDIR) $(BINARIES)
	@if [ -r cheetah.so ];  then cp -f cheetah.so  $(OUTDIR); fi
	@if [ -r cheetah.dll ]; then cp -f cheetah.dll $(OUTDIR); fi

$(BINARIES) : % : $(OUTDIR)/cheetah.o %.o
	@echo "    Linking $@"
	@$(LD) -o $@ $^ $(SYSLIBS)
    

#==========================================================================
# RULES
#==========================================================================
$(OUTDIR)/%.o : %.c
	@echo "+++ Compiling $<"
	@$(CC) $(CFLAGS) -c -o $@ $<

$(OUTDIR) :
	@echo "=== Making output directory."
	-@mkdir -p $(OUTDIR)

$(OUT_LIBS) : $(LIBS)
	@echo "=== Copying libraries to output directory."
	-@cp -f $^ $(OUTDIR)

clean:
	@echo "=== Cleaning output directory."
	@rm -fr $(OUTDIR)
	@rm -f MSVC/*.ncb MSVC/*.opt MSVC/*.plg
	@rm -f .bak* *~
