// My user-defined exception handlings
#include <string>

class DivisionbByZero { // Author: Xiaozhu
    public:
        DivisionbByZero() { message = " Error 0: Division by zero"; } // Author: Xiaozhu
        DivisionbByZero( std::string str ) { message = str; } // Author: Xiaozhu
        std::string what() { return message; } // Author: Xiaozhu
    private:
        std::string message;
};

class NotPositive {
    public:
        NotPositive() { message = " Error 4: The element must be Positive"; } // Author: Xiaozhu
        NotPositive( std::string str ) { message = str; } // Author: Xiaozhu
        std::string what() { return message; } // Author: Xiaozhu
    private:
        std::string message;
};

class NotExist {
    public:
        NotExist() { message = " Error 6: The instance does not exist"; } // Author: Xiaozhu
        NotExist( std::string str ) { message = str; } // Author: Xiaozhu
        std::string what() { return message; } // Author: Xiaozhu
    private:
        std::string message;
};

class IsNullptr {
    public:
        IsNullptr() { message = " Error 5: The instance is not assigned"; } // Author: Xiaozhu
        IsNullptr( std::string str ) { message = str; } // Author: Xiaozhu
        std::string what() { return message; } // Author: Xiaozhu
    private:
        std::string message;
};

class DimensionNEqual { // Author: Xiaozhu
    public:
        DimensionNEqual() { // Author: Xiaozhu
            message = " Error 1: Column of A and Row of B are not equal! ";
        }
        DimensionNEqual( std::string str ) { message = str; } // Author: Xiaozhu
        std::string what() { return message; } // Author: Xiaozhu
    private:
        std::string message; // Author: Xiaozhu
};

class MatrixIsNULL { // Author: Xiaozhu
/*
* It is recommended that All dynamic Arraies are assigned NULL when declared, and it is
* better if they assigned as NULL after destracted. 
* This user-defined exception class is mainly used to indecate whether the dynamic array
* is NULL or not. 
* The default constructor MatrixIsNULL() only display general perpose of this class. For 
* custom error message, use  MatrixIsNULL( string str ) instead.
*/
    public:
        MatrixIsNULL () { // Author: Xiaozhu
            message = "Error 2: Matrix does not exist!"; // Author: Xiaozhu
        }
        MatrixIsNULL( std::string str ) { message = str; } // Author: Xiaozhu
        std::string what() { return message; } // Author: Xiaozhu
    private:
        std::string message; // Author: Xiaozhu
};
/*
* The user-defined excaption class InvalidDimension is used to indcate the wrong demension
* problem of dynamic metrics. e.g. when using A = B, the agreement of their dimensions 
* should be checked seperately.
* All demension related problems or errors should reference to this class. and you can throw
*  custom error message by calling InvalidDimension( string str ).
*/
class InvalidDimension { // Author: Xiaozhu
    public:
        InvalidDimension () { // Author: Xiaozhu
            message = "Error 3: Dimension is invalid!"; // Author: Xiaozhu
        }
        InvalidDimension( std::string str ) { message = str; } // Author: Xiaozhu// Author: Xiaozhu
        std::string what() { return message; } // Author: Xiaozhu
    private:
        std::string message; // Author: Xiaozhu
};