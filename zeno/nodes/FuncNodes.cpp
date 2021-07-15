#include <zeno/zeno.h>
#include <zeno/DictObject.h>
#include <zeno/FunctionObject.h>
#include <zeno/ConditionObject.h>
#include <zeno/ContextManaged.h>
#include <cassert>


struct FuncBegin : zeno::INode {
    zeno::FunctionObject::DictType m_args;

    virtual void apply() override {
        auto args = std::make_shared<zeno::DictObject>();
        if (has_input("extraArgs")) {
            auto extraArgs = get_input<zeno::DictObject>("extraArgs");
            for (auto const &[key, ptr]: extraArgs->lut) {
                args->lut[key] = ptr;
            }
        }
        for (auto const &[key, ptr]: m_args) {
            args->lut[key] = ptr;
        }
        set_output("args", std::move(args));
        set_output("FUNC", std::make_shared<zeno::ConditionObject>());
    }
};

ZENDEFNODE(FuncBegin, {
    {"extraArgs"},
    {"args", "FUNC"},
    {},
    {"functional"},
});


struct FuncEnd : zeno::ContextManagedNode {
    virtual void doApply() override {
        auto [sn, ss] = inputBounds.at("FUNC");
        auto fore = dynamic_cast<FuncBegin *>(graph->nodes.at(sn).get());
        if (!fore) {
            printf("FuncEnd::FUNC must be conn to FuncBegin::FUNC!\n");
            abort();
        }
        graph->applyNode(sn);
        auto func = std::make_shared<zeno::FunctionObject>();
        func->func = [this, fore] (zeno::FunctionObject::DictType const &args) {
            fore->m_args = args;
            push_context();
            zeno::INode::doApply();
            pop_context();
            zeno::FunctionObject::DictType rets{};
            if (has_input("rets")) {
                auto frets = get_input<zeno::DictObject>("rets");
                rets = frets->lut;
            }
            return rets;
        };
        set_output("function", std::move(func));
    }

    virtual void apply() override {}
};

ZENDEFNODE(FuncEnd, {
    {"rets", "FUNC"},
    {"function"},
    {},
    {"functional"},
});


struct FuncCall : zeno::ContextManagedNode {
    virtual void apply() override {
        auto func = get_input<zeno::FunctionObject>("func");

        zeno::FunctionObject::DictType args{};
        if (has_input("args") {
            auto fargs = get_input<zeno::DictObject>("args");
            args = fargs->lut;
        }
    }
};

ZENDEFNODE(FuncCall, {
    {"function", "args"},
    {"rets"},
    {},
    {"functional"},
});
