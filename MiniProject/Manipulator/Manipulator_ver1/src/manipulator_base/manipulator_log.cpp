//
// Created by lacie on 29/08/2019.
//

#include "../../include/manipulator_base/manipulator_log.h"

void manipulator_base::log::print(STRING str, STRING color)
{
    if(color == "RED")
        printf(ANSI_COLOR_RED);
    else if(color == "GREEN")
        printf(ANSI_COLOR_GREEN);
    else if(color == "YELLOW")
        printf(ANSI_COLOR_YELLOW);
    else if(color == "BLUE")
        printf(ANSI_COLOR_BLUE);
    else if(color == "MAGENTA")
        printf(ANSI_COLOR_MAGENTA);
    else if(color == "CYAN")
        printf(ANSI_COLOR_CYAN);

    printf("%s", str.c_str());
    printf(ANSI_COLOR_RESET);
}

void manipulator_base::log::print(STRING str, double data, uint8_t decimal_point, STRING color)
{
    if(color == "RED")
        printf(ANSI_COLOR_RED);
    else if(color == "GREEN")
        printf(ANSI_COLOR_GREEN);
    else if(color == "YELLOW")
        printf(ANSI_COLOR_YELLOW);
    else if(color == "BLUE")
        printf(ANSI_COLOR_BLUE);
    else if(color == "MAGENTA")
        printf(ANSI_COLOR_MAGENTA);
    else if(color == "CYAN")
        printf(ANSI_COLOR_CYAN);
    printf("%s %.*lf", str.c_str(), decimal_point, data);
    printf(ANSI_COLOR_RESET);
}

void manipulator_base::log::print(const char *str, STRING color)
{
    if(color == "RED")
        printf(ANSI_COLOR_RED);
    else if(color == "GREEN")
        printf(ANSI_COLOR_GREEN);
    else if(color == "YELLOW")
        printf(ANSI_COLOR_YELLOW);
    else if(color == "BLUE")
        printf(ANSI_COLOR_BLUE);
    else if(color == "MAGENTA")
        printf(ANSI_COLOR_MAGENTA);
    else if(color == "CYAN")
        printf(ANSI_COLOR_CYAN);
    printf("%s", str);
    printf(ANSI_COLOR_RESET);
}

void manipulator_base::log::print(const char *str, double data, uint8_t decimal_point, STRING color)
{
    if(color == "RED")
        printf(ANSI_COLOR_RED);
    else if(color == "GREEN")
        printf(ANSI_COLOR_GREEN);
    else if(color == "YELLOW")
        printf(ANSI_COLOR_YELLOW);
    else if(color == "BLUE")
        printf(ANSI_COLOR_BLUE);
    else if(color == "MAGENTA")
        printf(ANSI_COLOR_MAGENTA);
    else if(color == "CYAN")
        printf(ANSI_COLOR_CYAN);
    printf("%s %.*lf", str, decimal_point, data);
    printf(ANSI_COLOR_RESET);
}

void manipulator_base::log::println(STRING str, STRING color)
{
    if(color == "RED")
        printf(ANSI_COLOR_RED);
    else if(color == "GREEN")
        printf(ANSI_COLOR_GREEN);
    else if(color == "YELLOW")
        printf(ANSI_COLOR_YELLOW);
    else if(color == "BLUE")
        printf(ANSI_COLOR_BLUE);
    else if(color == "MAGENTA")
        printf(ANSI_COLOR_MAGENTA);
    else if(color == "CYAN")
        printf(ANSI_COLOR_CYAN);
    printf("%s\n", str.c_str());
    printf(ANSI_COLOR_RESET);
}

void manipulator_base::log::println(STRING str, double data, uint8_t decimal_point, STRING color)
{
    if(color == "RED")
        printf(ANSI_COLOR_RED);
    else if(color == "GREEN")
        printf(ANSI_COLOR_GREEN);
    else if(color == "YELLOW")
        printf(ANSI_COLOR_YELLOW);
    else if(color == "BLUE")
        printf(ANSI_COLOR_BLUE);
    else if(color == "MAGENTA")
        printf(ANSI_COLOR_MAGENTA);
    else if(color == "CYAN")
        printf(ANSI_COLOR_CYAN);
    printf("%s %.*lf\n", str.c_str(), decimal_point, data);
    printf(ANSI_COLOR_RESET);
}

void manipulator_base::log::println(const char *str, STRING color)
{
    if(color == "RED")
        printf(ANSI_COLOR_RED);
    else if(color == "GREEN")
        printf(ANSI_COLOR_GREEN);
    else if(color == "YELLOW")
        printf(ANSI_COLOR_YELLOW);
    else if(color == "BLUE")
        printf(ANSI_COLOR_BLUE);
    else if(color == "MAGENTA")
        printf(ANSI_COLOR_MAGENTA);
    else if(color == "CYAN")
        printf(ANSI_COLOR_CYAN);
    printf("%s\n", str);
    printf(ANSI_COLOR_RESET);
}

void manipulator_base::log::println(const char *str, double data, uint8_t decimal_point, STRING color)
{
    if(color == "RED")
        printf(ANSI_COLOR_RED);
    else if(color == "GREEN")
        printf(ANSI_COLOR_GREEN);
    else if(color == "YELLOW")
        printf(ANSI_COLOR_YELLOW);
    else if(color == "BLUE")
        printf(ANSI_COLOR_BLUE);
    else if(color == "MAGENTA")
        printf(ANSI_COLOR_MAGENTA);
    else if(color == "CYAN")
        printf(ANSI_COLOR_CYAN);
    printf("%s %.*lf\n", str, decimal_point, data);
    printf(ANSI_COLOR_RESET);
}

void manipulator_base::log::info(STRING str)
{
    printf("[INFO] %s\n", str.c_str());
}

void manipulator_base::log::info(STRING str, double data, uint8_t decimal_point)
{
    printf("[INFO] %s %.*lf\n", str.c_str(), decimal_point, data);
}

void manipulator_base::log::info(const char *str)
{
    printf("[INFO] %s\n", str);
}

void manipulator_base::log::info(const char *str, double data, uint8_t decimal_point)
{
    printf("[INFO] %s %.*lf\n", str, decimal_point, data);
}

void manipulator_base::log::warn(STRING str)
{
    printf(ANSI_COLOR_YELLOW);
    printf("[WARN] %s\n", str.c_str());
    printf(ANSI_COLOR_RESET);
}

void manipulator_base::log::warn(STRING str, double data, uint8_t decimal_point)
{
    printf(ANSI_COLOR_YELLOW);
    printf("[WARN] %s %.*lf\n",str.c_str(), decimal_point, data);
    printf(ANSI_COLOR_RESET);
}

void manipulator_base::log::warn(const char *str)
{
    printf(ANSI_COLOR_YELLOW);
    printf("[WARN] %s\n", str);
    printf(ANSI_COLOR_RESET);
}

void manipulator_base::log::warn(const char *str, double data, uint8_t decimal_point)
{
    printf(ANSI_COLOR_YELLOW);
    printf("[WARN] %s %.*lf\n", str, decimal_point, data);
    printf(ANSI_COLOR_RESET);
}

void manipulator_base::log::error(STRING str)
{
    printf(ANSI_COLOR_RED);
    printf("[ERROR] %s\n", str.c_str());
    printf(ANSI_COLOR_RESET);
}

void manipulator_base::log::error(STRING str, double data, uint8_t decimal_point)
{
    printf(ANSI_COLOR_RED);
    printf("[ERROR] %s %.*lf\n", str.c_str(), decimal_point, data);
    printf(ANSI_COLOR_RESET);
}

void manipulator_base::log::error(const char *str)
{
    printf(ANSI_COLOR_RED);
    printf("[ERROR] %s\n", str);
    printf(ANSI_COLOR_RESET);
}

void manipulator_base::log::error(const char *str, double data, uint8_t decimal_point)
{
    printf(ANSI_COLOR_RED);
    printf("[ERROR] %s %.*lf\n", str, decimal_point, data);
    printf(ANSI_COLOR_RESET);
}









