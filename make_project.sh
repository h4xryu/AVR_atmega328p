echo "Enter the project name : "
read project_name
@echo off
mkdir ./${project_name}
cd ${project_name}
cp ../Makefile_default Makefile
touch ${project_name}.cpp



echo "#ifndef F_CPU
#define F_CPU 16000000UL // or whatever may be your frequency
#endif
#include <avr/io.h>
#include <util/delay.h>

int main(void){
	
	while(1)
	{	

    }
    return 0;
}" > ${project_name}.cpp

	


exit
