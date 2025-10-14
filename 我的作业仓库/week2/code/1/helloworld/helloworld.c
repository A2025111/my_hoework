#define _CRT_SECURE_NO_WARNINGS
#include<stdio.h>

int main() {
	int input;
	while (1) {
		printf("«Î ‰»Î-1£¨0£¨1\n");
		scanf("%d", &input);
		if (input == -1) {
			printf("≥Ã–ÚÕÀ≥ˆ\n");
			break;
		}
		if (input == 0)
			printf("hello world\n");
		else if (input == 1)
			printf("HELLOWORLD");
	}
	return 0;
}