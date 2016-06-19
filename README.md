# kalman
El archivo "ratoekf" es el ejemplo del raton modificado por mi para añadirle el ekf
el otro archivo es probando la libreria para medir el tiempo, no la compila da el siguiente fallo:
patri@patri:~/tfm/filtro de kalman$ make
Scanning dependencies of target ejemplotiempo
[100%] Building CXX object CMakeFiles/ejemplotiempo.dir/ejemplotiempo.cpp.o
In file included from /usr/include/c++/4.8/chrono:35:0,
                 from /home/patri/tfm/filtro de kalman/ejemplotiempo.cpp:4:
/usr/include/c++/4.8/bits/c++0x_warning.h:32:2: error: #error This file requires compiler and library support for the ISO C++ 2011 standard. This support is currently experimental, and must be enabled with the -std=c++11 or -std=gnu++11 compiler options.
 #error This file requires compiler and library support for the \
  ^
/home/patri/tfm/filtro de kalman/ejemplotiempo.cpp: In function ‘int main()’:
/home/patri/tfm/filtro de kalman/ejemplotiempo.cpp:9:15: error: ‘size’ does not name a type
     for (auto size = 1ull; size < 1000000000ull; size *= 100) {
               ^
/home/patri/tfm/filtro de kalman/ejemplotiempo.cpp:9:28: error: expected ‘;’ before ‘size’
     for (auto size = 1ull; size < 1000000000ull; size *= 100) {
                            ^
/home/patri/tfm/filtro de kalman/ejemplotiempo.cpp:9:28: error: ‘size’ was not declared in this scope
/home/patri/tfm/filtro de kalman/ejemplotiempo.cpp:11:14: error: ‘start’ does not name a type
         auto start = std::chrono::high_resolution_clock::now();
              ^
/home/patri/tfm/filtro de kalman/ejemplotiempo.cpp:16:14: error: ‘end’ does not name a type
         auto end = std::chrono::high_resolution_clock::now();
              ^
/home/patri/tfm/filtro de kalman/ejemplotiempo.cpp:17:14: error: ‘std::chrono’ has not been declared
         std::chrono::duration<double> diff = end-start;
              ^
/home/patri/tfm/filtro de kalman/ejemplotiempo.cpp:17:31: error: expected primary-expression before ‘double’
         std::chrono::duration<double> diff = end-start;
                               ^
/home/patri/tfm/filtro de kalman/ejemplotiempo.cpp:17:31: error: expected ‘;’ before ‘double’
/home/patri/tfm/filtro de kalman/ejemplotiempo.cpp:19:44: error: ‘diff’ was not declared in this scope
                   << size << " ints : " << diff.count() << " s\n";
                                            ^
make[2]: *** [CMakeFiles/ejemplotiempo.dir/ejemplotiempo.cpp.o] Error 1
make[1]: *** [CMakeFiles/ejemplotiempo.dir/all] Error 2
make: *** [all] Error 2
patri@patri:~/tfm/filtro de kalman$
