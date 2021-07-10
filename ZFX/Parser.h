#pragma once

#include "AST.h"

struct Parser {
    std::vector<std::string> tokens;

    explicit Parser
        ( std::vector<std::string> const &tokens_
        )
        : tokens(tokens_)
    {
    }

    AST::Ptr parse_atom(Iter iter) {
        if (auto s = *iter; is_atom(s)) {
            return make_ast(s, iter + 1);
        }
        return nullptr;
    }

    AST::Ptr parse_operator(Iter iter, std::set<std::string> const &allows) {
        if (auto s = *iter; contains(allows, s)) {
            return make_ast(s, iter + 1);
        }
        return nullptr;
    }

    AST::Ptr parse_factor(Iter iter) {
        if (auto a = parse_atom(iter); a) {
            return a;
        }
        if (auto ope = parse_operator(iter, {"+", "-"}); ope) {
            if (auto rhs = parse_factor(ope->iter); rhs) {
                return make_ast(ope->token, rhs->iter, {std::move(rhs)});
            }
        }
        if (auto ope = parse_operator(iter, {"("}); ope) {
            if (auto rhs = parse_expr(ope->iter); rhs) {
                if (auto ket = parse_operator(rhs->iter, {")"}); ket) {
                    rhs->iter = ket->iter;
                    return rhs;
                }
            }
        }
        return nullptr;
    }

    AST::Ptr parse_term(Iter iter) {
        if (auto lhs = parse_factor(iter); lhs) {
            while (1) if (auto ope = parse_operator(lhs->iter, {"*", "/", "%"}); ope) {
                if (auto rhs = parse_factor(ope->iter); rhs) {
                    lhs = make_ast(ope->token, rhs->iter, {std::move(lhs), std::move(rhs)});
                }
            } else break;
            return lhs;
        }
        return nullptr;
    }

    AST::Ptr parse_expr(Iter iter) {
        if (auto lhs = parse_term(iter); lhs) {
            while (1) if (auto ope = parse_operator(lhs->iter, {"+", "-"}); ope) {
                    if (auto rhs = parse_term(ope->iter); rhs) {
                        lhs = make_ast(ope->token, rhs->iter, {std::move(lhs), std::move(rhs)});
                    }
            } else break;
            return lhs;
        }
        return nullptr;
    }

    AST::Ptr parse_stmt(Iter iter) {
        if (auto lhs = parse_atom(iter); lhs) {
            if (auto ope = parse_operator(lhs->iter, {"="}); ope) {
                if (auto rhs = parse_expr(ope->iter); rhs) {
                    return make_ast(ope->token, rhs->iter, {std::move(lhs), std::move(rhs)});
                }
            }
        }
        return nullptr;
    }

    auto parse() {
        std::vector<AST::Ptr> asts;
        Iter iter = tokens.begin();
        while (iter != tokens.end()) {
            auto p = parse_stmt(iter);
            if (!p) break;
            iter = p->iter;
            asts.push_back(std::move(p));
        }
        return asts;
    }
};