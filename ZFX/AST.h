#pragma once

#include "common.h"

using Iter = typename std::vector<std::string>::iterator;

struct AST {
    using Ptr = copiable_unique_ptr<AST>;

    Iter iter;
    std::string token;
    std::vector<AST::Ptr> args;

    explicit AST
        ( std::string const &token_
        , Iter iter_
        , std::vector<AST::Ptr> const &args_ = {}
        )
        : token(std::move(token_))
        , iter(std::move(iter_))
        , args(std::move(args_))
        {}

    void print() {
        if (args.size())
            cout << '(';
        cout << token;
        for (auto const &a: args) {
            cout << ' ';
            a->print();
        }
        if (args.size())
            cout << ')';
    }
};

inline AST::Ptr make_ast(std::string const &token, Iter iter, std::vector<AST::Ptr> const &args = {}) {
    return std::make_unique<AST>(token, iter, args);
}