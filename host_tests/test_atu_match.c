// Host test (2026-11-12). Build and run on a PC, never in the Arduino sketch folder:
//   gcc -std=c11 -Wall -Wextra -O1 test_atu_match.c -o test_atu_match && ./test_atu_match
// Mirror of serial_commands.cpp brace_try_atu()'s matching, checked against the ten commands the plugin's ATU dialog sends.
#include <stdio.h>
#include <string.h>
static int match(char letter, const char *arg) {
    static const char *const k_atu[] = { "+l", "-l", "+c", "-c", "i", "o", "s", "f", "?", "b" };
    char body[4];
    if (strlen(arg) > 1) return 0;
    body[0] = letter; strcpy(&body[1], arg);
    for (unsigned i = 0; i < sizeof k_atu / sizeof k_atu[0]; i++) if (!strcmp(body, k_atu[i])) return 1;
    return 0;
}
int main(void) {
    struct { const char *cmd; int want; } t[] = {
        {"+l",1},{"-l",1},{"+c",1},{"-c",1},{"i",1},{"o",1},{"s",1},{"f",1},{"?",1},{"b",1},
        {"F",0},{"B",0},{"F14200000",0},{"B3",0},{"L50",0},{"M8",0},{"I",0},{"O",0},{"S",0},
        {"+x",0},{"-",0},{"+",0},{"ff",0},{"i1",0},{"Z0123",0},{"X1",0},{"T54",0},
    };
    int bad = 0;
    for (unsigned i = 0; i < sizeof t / sizeof t[0]; i++) {
        int got = match(t[i].cmd[0], t[i].cmd + 1);
        if (got != t[i].want) { printf("FAIL {%s}: got %d want %d\n", t[i].cmd, got, t[i].want); bad++; }
    }
    printf("%u cases, %d failed\n", (unsigned)(sizeof t / sizeof t[0]), bad);
    return bad != 0;
}
