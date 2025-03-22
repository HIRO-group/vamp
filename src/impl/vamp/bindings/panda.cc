#include <vamp/bindings/common.hh>
#include <vamp/bindings/init.hh>
#include <vamp/robots/panda.hh>

void vamp::binding::init_panda(nanobind::module_ &pymodule)
{
    using namespace nanobind::literals;

    auto submodule = vamp::binding::init_robot<vamp::robots::Panda>(pymodule);

    submodule.def(
        "set_joint_limits",
        &vamp::robots::Panda::set_joint_limits,
        "lower"_a,
        "upper"_a,
        "Sets the joint limits for the Panda robot."
    );
}
