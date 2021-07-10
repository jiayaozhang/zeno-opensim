#pragma once

#include "Statement.h"

struct IR {
    std::vector<std::unique_ptr<Statement>> stmts;

    template <class T, class ...Ts>
    T *emplace_back(Ts &&...ts) {
        auto id = stmts.size();
        auto stmt = std::make_unique<T>(id, std::forward<Ts>(ts)...);
        auto raw_ptr = stmt.get();
        stmts.push_back(std::move(stmt));
        return raw_ptr;
    }

    void print() const {
        for (auto const &s: stmts) {
            cout << s->print() << endl;
        }
    }
};