// Ali B.
// Function template declaration for consoleLog.
// This function is designed to print multiple arguments to the Serial monitor.
// It utilizes variadic templates to handle any number of arguments.

template<typename T, typename... Args>
void consoleLog(T first, Args... args) {
    // Print the first argument to the Serial monitor.
    Serial.print(first);
    // Print a space after each argument.
    Serial.print("");
    // Recursively call consoleLog with the remaining arguments.
    consoleLog(args...);
}
// Overloaded function for the case when there are no arguments left.
// It simply prints a newline character to the Serial monitor.
void consoleLog() {
    Serial.println();
}
