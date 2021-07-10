#include "Tokenizer.h"

std::vector<std::string> tokenize(const char *cp) {
    std::vector<std::string> tokens;
    while (1) {
        for (; *cp && isspace(*cp); cp++);
        if (!*cp)
            break;

        if (isalpha(*cp) || strchr("_$@", *cp)) {
            std::string res;
            for (; isalnum(*cp) || *cp && strchr("_$@", *cp); cp++)
                res += *cp;
            tokens.push_back(res);

        } else if (isdigit(*cp) || *cp == '-' && isdigit(cp[1])) {
            std::string res;
            for (; isdigit(*cp) || *cp && strchr(".e-", *cp); cp++)
                res += *cp;
            tokens.push_back(res);

        } else if (strchr(opchars, *cp)) {
            std::string res;
            res += *cp++;
            tokens.push_back(res);

        } else {
            error("unexpected character token: `%c`", *cp);
            break;
        }
    }
    tokens.push_back("");  // EOF sign
    return tokens;
}