#  Task_1
__1. To write a C-language code, compile it.__
   
   Open leafpad text editor and name the file using command,  __*leafpad sum1ton.c*__
![Screenshot 2024-05-24 162737](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/a88ebd77-c1cf-46d3-94e4-0487a5c11e20)

   Compile it using command ,__*gcc sum1ton.c*__  
   
__2. To convert the C-code into assembly language using RISCV tools.__  

Use command  __*riscv64-unknown-elf-gcc -o1 -mabi-lp64 -march=rv64i -o sumiton.o sumiton.c*__  to generate assembly level code using __*o1*__ and  __*ofast*__  type processes.  
![Screenshot 2024-05-24 161132](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/5cc0c0dc-3d34-4627-86f7-dcc22fe3ed17)

Get the results using command __riscv64-unknown-elf-objdump -d sum1ton.0 | less__  
![Screenshot 2024-05-24 161207](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/6466e404-ba53-4d82-b5c4-27333e8ed077)

Output of main section of the file sum1ton.c in both processes came out to be same.   

Got 35 instructions inside the main section of code.
![Screenshot 2024-05-24 161028](https://github.com/princesuman2004/VSD_Mini_Internship/assets/128327318/d8be156e-460b-422c-9770-5469d525b1cb)
     
