/* GRAMPC 问题描述:LL100 6-DOF 浮动基座机械臂 NMPC IK + 避障。
 *
 * 状态 x = q ∈ R^nq,控制 u = q̇ ∈ R^nq,动力学 ẋ = u。
 * 参数 p = [目标位置(3), 目标旋转按行展开(9)] ∈ R^12。
 * 代价 / 约束的所有非平凡导数来自 CasADi 生成的 C 函数(ll100_grampc_gen.c)。
 *
 * 约束约定:hfct(x,u,p) <= 0;由关节限位 + 避障 margin 组成(见 grampc_codegen.py)。
 */

#include "problem_description.hpp"

extern "C" {
#include "ll100_grampc_gen.h"
}

#include <vector>
#include <typeindex>
#include <pybind11/pybind11.h>

using namespace grampc;

namespace {

// CasADi 生成函数的统一调用 helper。每个函数自带 *_work 报告所需缓冲尺寸。
struct CasadiFn {
    typedef int (*eval_t)(const casadi_real**, casadi_real**, casadi_int*, casadi_real*, int);
    typedef int (*work_t)(casadi_int*, casadi_int*, casadi_int*, casadi_int*);

    eval_t eval;
    casadi_int sz_arg = 0, sz_res = 0, sz_iw = 0, sz_w = 0;
    std::vector<const casadi_real*> arg;
    std::vector<casadi_real*> res;
    std::vector<casadi_int> iw;
    std::vector<casadi_real> w;

    void init(eval_t e, work_t wfn) {
        eval = e;
        wfn(&sz_arg, &sz_res, &sz_iw, &sz_w);
        arg.assign(sz_arg, nullptr);
        res.assign(sz_res, nullptr);
        iw.assign(sz_iw > 0 ? sz_iw : 1, 0);
        w.assign(sz_w > 0 ? sz_w : 1, 0.0);
    }

    // 调用:inputs 为各输入指针,outputs 为各输出指针(由调用方提供存储)。
    void call(std::initializer_list<const casadi_real*> inputs,
              std::initializer_list<casadi_real*> outputs) {
        casadi_int i = 0;
        for (const casadi_real* in : inputs) { arg[i++] = in; }
        i = 0;
        for (casadi_real* out : outputs) { res[i++] = out; }
        eval(arg.data(), res.data(), iw.data(), w.data(), 0);
    }
};

void alias_pygrampc_problem_description() {
    namespace py = pybind11;

    py::object base = py::module_::import("pygrampc._core").attr("ProblemDescription");
    auto* base_info = py::detail::get_type_info(reinterpret_cast<PyTypeObject*>(base.ptr()));
    if (base_info == nullptr) {
        throw std::runtime_error("pygrampc._core.ProblemDescription is not registered");
    }

    auto& internals = py::detail::get_internals();
    auto local_type = std::type_index(typeid(ProblemDescription));
    if (internals.registered_types_cpp.find(local_type) == internals.registered_types_cpp.end()) {
        internals.registered_types_cpp[local_type] = base_info;
    }
#if PYBIND11_INTERNALS_VERSION >= 12
    internals.registered_types_cpp_fast[&typeid(ProblemDescription)] = base_info;
#endif
}

}  // namespace

class PYBIND11_EXPORT LL100Problem : public ProblemDescription {
public:
    LL100Problem(typeInt nx, typeInt nu, typeInt np, typeInt nh)
        : ProblemDescription() {
        Nx = nx;
        Nu = nu;
        Np = np;
        Ng = 0;
        Nh = nh;
        NgT = 0;
        NhT = 0;

        fn_l_.init(ll100_l, ll100_l_work);
        fn_dldx_.init(ll100_dldx, ll100_dldx_work);
        fn_dldu_.init(ll100_dldu, ll100_dldu_work);
        fn_V_.init(ll100_V, ll100_V_work);
        fn_dVdx_.init(ll100_dVdx, ll100_dVdx_work);
        fn_h_.init(ll100_h, ll100_h_work);
        fn_dhdx_vec_.init(ll100_dhdx_vec, ll100_dhdx_vec_work);
    }

    ~LL100Problem() {}

    /** ẋ = u **/
    void ffct(VectorRef out, ctypeRNum t, VectorConstRef x, VectorConstRef u, VectorConstRef p) override {
        for (typeInt i = 0; i < Nu; ++i) { out[i] = u[i]; }
    }
    /** (df/dx)^T * vec = 0(f 不依赖 x) **/
    void dfdx_vec(VectorRef out, ctypeRNum t, VectorConstRef x, VectorConstRef vec, VectorConstRef u, VectorConstRef p) override {
        for (typeInt i = 0; i < Nx; ++i) { out[i] = 0.0; }
    }
    /** (df/du)^T * vec = vec(f = u) **/
    void dfdu_vec(VectorRef out, ctypeRNum t, VectorConstRef x, VectorConstRef vec, VectorConstRef u, VectorConstRef p) override {
        for (typeInt i = 0; i < Nu; ++i) { out[i] = vec[i]; }
    }

    /** 运行代价 l **/
    void lfct(VectorRef out, ctypeRNum t, VectorConstRef x, VectorConstRef u, VectorConstRef p, VectorConstRef xdes, VectorConstRef udes) override {
        fn_l_.call({x.data(), u.data(), p.data(), xdes.data(), udes.data()}, {out.data()});
    }
    void dldx(VectorRef out, ctypeRNum t, VectorConstRef x, VectorConstRef u, VectorConstRef p, VectorConstRef xdes, VectorConstRef udes) override {
        fn_dldx_.call({x.data(), u.data(), p.data(), xdes.data(), udes.data()}, {out.data()});
    }
    void dldu(VectorRef out, ctypeRNum t, VectorConstRef x, VectorConstRef u, VectorConstRef p, VectorConstRef xdes, VectorConstRef udes) override {
        fn_dldu_.call({x.data(), u.data(), p.data(), xdes.data(), udes.data()}, {out.data()});
    }

    /** 终端代价 V **/
    void Vfct(VectorRef out, ctypeRNum T, VectorConstRef x, VectorConstRef p, VectorConstRef xdes) override {
        fn_V_.call({x.data(), p.data(), xdes.data()}, {out.data()});
    }
    void dVdx(VectorRef out, ctypeRNum T, VectorConstRef x, VectorConstRef p, VectorConstRef xdes) override {
        fn_dVdx_.call({x.data(), p.data(), xdes.data()}, {out.data()});
    }
    void dVdT(VectorRef out, ctypeRNum T, VectorConstRef x, VectorConstRef p, VectorConstRef xdes) override {
        out[0] = 0.0;
    }

    /** 不等式约束 h <= 0 **/
    void hfct(VectorRef out, ctypeRNum t, VectorConstRef x, VectorConstRef u, VectorConstRef p) override {
        fn_h_.call({x.data(), u.data(), p.data()}, {out.data()});
    }
    void dhdx_vec(VectorRef out, ctypeRNum t, VectorConstRef x, VectorConstRef u, VectorConstRef p, VectorConstRef vec) override {
        fn_dhdx_vec_.call({x.data(), u.data(), p.data(), vec.data()}, {out.data()});
    }
    void dhdu_vec(VectorRef out, ctypeRNum t, VectorConstRef x, VectorConstRef u, VectorConstRef p, VectorConstRef vec) override {
        for (typeInt i = 0; i < Nu; ++i) { out[i] = 0.0; }  // h 不依赖 u
    }

private:
    CasadiFn fn_l_, fn_dldx_, fn_dldu_, fn_V_, fn_dVdx_, fn_h_, fn_dhdx_vec_;
};

PYBIND11_MODULE(ll100_grampc_problem, m) {
    alias_pygrampc_problem_description();

    pybind11::class_<LL100Problem, ProblemDescription, std::shared_ptr<LL100Problem>>(m, "LL100Problem")
        .def(pybind11::init<typeInt, typeInt, typeInt, typeInt>())
        .def_readonly("Nx", &LL100Problem::Nx)
        .def_readonly("Nu", &LL100Problem::Nu)
        .def_readonly("Np", &LL100Problem::Np)
        .def_readonly("Ng", &LL100Problem::Ng)
        .def_readonly("Nh", &LL100Problem::Nh)
        .def_readonly("NgT", &LL100Problem::NgT)
        .def_readonly("NhT", &LL100Problem::NhT)

        .def("ffct", &LL100Problem::ffct)
        .def("dfdx_vec", &LL100Problem::dfdx_vec)
        .def("dfdu_vec", &LL100Problem::dfdu_vec)
        .def("dfdp_vec", &LL100Problem::dfdp_vec)

        .def("lfct", &LL100Problem::lfct)
        .def("dldx", &LL100Problem::dldx)
        .def("dldu", &LL100Problem::dldu)
        .def("dldp", &LL100Problem::dldp)

        .def("Vfct", &LL100Problem::Vfct)
        .def("dVdx", &LL100Problem::dVdx)
        .def("dVdp", &LL100Problem::dVdp)
        .def("dVdT", &LL100Problem::dVdT)

        .def("gfct", &LL100Problem::gfct)
        .def("dgdx_vec", &LL100Problem::dgdx_vec)
        .def("dgdu_vec", &LL100Problem::dgdu_vec)
        .def("dgdp_vec", &LL100Problem::dgdp_vec)

        .def("hfct", &LL100Problem::hfct)
        .def("dhdx_vec", &LL100Problem::dhdx_vec)
        .def("dhdu_vec", &LL100Problem::dhdu_vec)
        .def("dhdp_vec", &LL100Problem::dhdp_vec)

        .def("gTfct", &LL100Problem::gTfct)
        .def("dgTdx_vec", &LL100Problem::dgTdx_vec)
        .def("dgTdp_vec", &LL100Problem::dgTdp_vec)
        .def("dgTdT_vec", &LL100Problem::dgTdT_vec)

        .def("hTfct", &LL100Problem::hTfct)
        .def("dhTdx_vec", &LL100Problem::dhTdx_vec)
        .def("dhTdp_vec", &LL100Problem::dhTdp_vec)
        .def("dhTdT_vec", &LL100Problem::dhTdT_vec)

        .def("dfdx", &LL100Problem::dfdx)
        .def("dfdxtrans", &LL100Problem::dfdxtrans)
        .def("dfdt", &LL100Problem::dfdt)
        .def("dHdxdt", &LL100Problem::dHdxdt)
        .def("Mfct", &LL100Problem::Mfct)
        .def("Mtrans", &LL100Problem::Mtrans);
}
