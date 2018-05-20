

void init_platform();


void main()
{
	static int counter = 0;
	int i = 0;
	int j = 0;
	init_platform();
	printk("Non-Secure bare metal VM: running ... \n\r");

	while(1){
		printk("Hello World fardin %d\n\r", counter++);

		for(i = 0; i < 2000; i++){
			for(j = 0; j < 1000; j++){
				/* Do nothing */
			}
		}
	}

	// END!!!
	while(1);
}


void init_platform(){

}
