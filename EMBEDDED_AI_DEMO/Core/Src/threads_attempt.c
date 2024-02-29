/*
 * threads_attempt.c
 *
 *  Created on: Feb 5, 2024
 *      Author: wardawg
 */

#include <stdio.h>
#include <pthread.h>
extern void initialise_monitor_handles(void);
void* task() {
	printf("We threading in this MF like we prime Ozil \n");
	return NULL;
}

int main() {
	initialise_monitor_handles();
	pthread_t thread1;
	pthread_create(&thread1, NULL, task, NULL);
	pthread_join(thread1, NULL);
	return 0;
}
