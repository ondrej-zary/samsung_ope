#CFLAGS = --std=gnu99 -Wall -Wextra -g -DDEBUG
CFLAGS = --std=gnu99 -Wall -Wextra -O2

samsung_ope: samsung_ope.c
	gcc $(CFLAGS) samsung_ope.c -lncurses -o samsung_ope

clean:
	rm -f samsung_ope
