CCOMP = gcc
#CFLAGS = -O4 -DNDEBUG -DEXCESS_TYPE_LONG -DPRINT_STAT -DCHECK_SOLUTION -Wall -lm
CFLAGS = -g -DPRINT_FLOW -DEXCESS_TYPE_LONG -DPRINT_STAT -DCHECK_SOLUTION -Wall -lm

all: hi_rankmincut 
hi_rankmincut: main.c hi_rankmincut.c parser_rankmincut.c timer.c
	$(CCOMP) $(CFLAGS) -o main_rankmincut main.c libm.so 
clean: 
	rm -f main_rankmincut *~
