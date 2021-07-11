#include "IRVisitor.h"
#include "Stmts.h"

namespace zfx {

struct TypeCheck : Visitor<TypeCheck> {
    using visit_stmt_types = std::tuple
        < AssignStmt
        , LiterialStmt
        , SymbolStmt
        , BinaryOpStmt
        , UnaryOpStmt
        , Statement
        >;

    std::unique_ptr<IR> ir = std::make_unique<IR>();

    void visit(Statement *stmt) {
    }

    void visit(SymbolStmt *stmt) {
        stmt->dim = stmt->symids.size();
    }

    void visit(LiterialStmt *stmt) {
        stmt->dim = 1;
    }

    void visit(UnaryOpStmt *stmt) {
        stmt->dim = stmt->src->dim;
    }

    void visit(BinaryOpStmt *stmt) {
        if (stmt->lhs->dim > 1 && stmt->rhs->dim > 1
            && stmt->lhs->dim != stmt->rhs->dim) {
            error("dimension mismatch in binary op: %d != %d",
                stmt->lhs->dim, stmt->rhs->dim);
        }
        if (stmt->lhs->dim != 0 && stmt->rhs->dim != 0) {
            stmt->dim = std::max(stmt->lhs->dim, stmt->rhs->dim);
        }
    }

    void visit(AssignStmt *stmt) {
        if (stmt->dst->dim == 0 && stmt->src->dim != 0) {
            stmt->dst->dim = stmt->src->dim;
            if (auto dst = dynamic_cast<TempSymbolStmt *>(stmt->dst); dst) {
                dst->symids.clear();
                dst->symids.resize(dst->dim, -1);
            }
        } else if (stmt->src->dim > 1 && stmt->dst->dim != stmt->src->dim) {
            error("dimension mismatch in assign: %d != %d",
                stmt->dst->dim, stmt->src->dim);
        }
        stmt->dim = stmt->dst->dim;
    }
};

void apply_type_check(IR *ir) {
    TypeCheck visitor;
    visitor.apply(ir);
}

}