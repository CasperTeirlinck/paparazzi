// This file is used to try out things in C++

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

using namespace std;

int main(){
    cout << "Hello world" << endl;
    cout << "4/5 = " << (float) 4 / 5 << endl ;

    // setting up the if statement 
    // == , != ....

    int age = 70;
    int ageAtLastExam = 16;
    bool isNotIntoxicated = true;

    if((age >= 1) && (age < 16)){

        cout << "You can't drive" << endl;
    } else if (! isNotIntoxicated){
        cout << "You can't drive" << endl;
    } else if (age >= 80 && ((age> 100) || ((age- ageAtLastExam) > 5))){
        cout << "You can't drive" << endl;
    } else {
        cout << "You can drive" << endl;
    }

    // creating a for loop
    /*
    for(int i =1; i <=10; i++){

        cout << i << endl;
    }
    */
    
// creating a while loop
    /*
    int randNum = (rand() % 100) +1;
    
    while(randNum != 100){

        cout << randNum << ",";
        randNum = (rand() % 100) +1;
    }

    cout << endl;
    */

    // working with pointers 

    int myAge = 39;
    int* agePtr = &myAge;

    cout << "Adress of pointer  " << agePtr << endl;
    cout << "Data at memory adress  " << *agePtr << endl;

    
 
    return 0;
}

