//
// Created by lacie on 29/08/2019.
//

#ifndef MANIPULATOR_VER1_MANIPULATOR_LOG_H
#define MANIPULATOR_VER1_MANIPULATOR_LOG_H

#include <unistd.h>
#include <vector>
#include <string>

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

typedef std::string STRING;

namespace manipulator_base
{
    namespace log
    {
        void print(STRING str, STRING color = "DEFAULT");
        void print(STRING str, double data, uint8_t decimal_point = 3, STRING color = " DEFAULT");
        void print(const char* str, STRING color = "DEFAULT");
        void print(const char* str, double data, uint8_t decimal_point = 3, STRING color = "DEFAULT");

        void println(STRING str, STRING color = "DEFAULT");
        void println(STRING str, double data, uint8_t decimal_point = 3, STRING color = "DEFAULT");
        void println(const char* str, STRING color = "DEFAULT");
        void println(const char* str, double data, uint8_t decimal_point = 3, STRING color = "DEFAULT");

        void info(STRING str);
        void info(STRING str, double data, uint8_t decimal_point = 3);
        void info(const char* str);
        void info(const char* str, double data, uint8_t decimal_point = 3);

        void warn(STRING str);
        void warn(STRING str, double data, uint8_t decimal_point = 3);
        void warn(const char* str);
        void warn(const char* str, double data, uint8_t decimal_point = 3);

        void error(STRING str);
        void error(STRING str, double data, uint8_t decimal_point = 3);
        void error(const char* str);
        void error(const char* str, double data, uint8_t decimal_point = 3);

        template <typename T> void print_vector(std::vector<T> &vec, uint8_t decimal_point = 3)
        {
            printf("(");
            for (uint8_t i = 0; i < vec.size(); i++)
            {
                printf("%.*lf", decimal_point, vec.at(i));
                if(i != (vec.size()-1))
                    printf(",");
                else
                    printf(")\n");
            }
        }

        template <typename vector> void print_vector(vector &vec, uint8_t decimal_point = 3)
        {
            printf("(");
            for (uint8_t i = 0; i < vec.size(); i++)
            {
                printf("%.*lf", decimal_point, vec(i));
                if(i != vec.size()-1)
                    printf(", ");
                else
                    printf(")\n");
            }
        }

        template <typename matrix> void print_matrix(matrix &m, uint8_t decimal_point = 3)
        {
            for (uint8_t i = 0; i < m.rows(); i++)
            {
                if(i == 0)
                    printf("(");
                else
                    printf(" ");
                for (uint8_t j = 0; j < m.cols(); j++)
                {
                    printf("%.*lf", decimal_point, m(i, j));
                    if(j != m.cols()-1)
                        printf(", ");
                }
                if(i != m.rows()-1)
                    printf("\n");
                else
                    printf(")\n");
            }
        }
    }
}

#endif //MANIPULATOR_VER1_MANIPULATOR_LOG_H
