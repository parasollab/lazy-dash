//   ht_diag scene.json
// Capability diagnostics: reach and valid IK per object pose, robot and grasp.
#include "lazydash/robot/UrIk.h"
#include "lazydash/scene/Scene.h"
#include "lazydash/schedule/MeshVerifier.h"
#include "lazydash/task/Capabilities.h"
#include "lazydash/task/TaskHypergraph.h"

#include <cstdio>
#include <iostream>

using namespace lazydash;

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "usage: ht_diag scene.json\n";
        return 2;
    }
    const Scene scene = Scene::load(argv[1]);
    const auto arms = scene.makeArms();
    const Capabilities caps(scene, arms);
    const auto meshes = scene.makeArms(Ur5eGeometry::Meshes);
    const MeshVerifier mesh(scene);
    std::vector<UrIk> ik;
    for (std::size_t r = 0; r < arms.size(); ++r) {
        ik.emplace_back(arms[r]);
        ik.back().setJointLimits(scene.arms[r].lower, scene.arms[r].upper);
    }
    const auto report = [&](const std::string &what, const Eigen::Isometry3d &pose) {
        std::printf("%s at (%.3f %.3f %.3f)\n", what.c_str(), pose.translation().x(), pose.translation().y(),
                    pose.translation().z());
        for (int r = 0; r < caps.numRobots(); ++r) {
            if (!caps.reaches(r, pose.translation())) {
                std::printf("  r%d: out of reach\n", r);
                continue;
            }
            for (int g = 0; g < caps.numGrasps(); ++g) {
                const auto &gs = caps.graspSet(r);
                int raw[2], ok[2], mesh_ok[2];
                const GraspSet::Stage stages[2] = {GraspSet::Stage::Grasp, GraspSet::Stage::Approach};
                for (int s = 0; s < 2; ++s) {
                    raw[s] = static_cast<int>(
                        ik[static_cast<std::size_t>(r)]
                            .solve(gs.eePose(static_cast<std::size_t>(g), pose, stages[s]), gs.eeLink())
                            .size());
                    ok[s] = static_cast<int>(caps.holdConfigs(r, g, pose, stages[s]).size());
                    mesh_ok[s] = 0;
                    for (const auto &q : ik[static_cast<std::size_t>(r)].solve(
                             gs.eePose(static_cast<std::size_t>(g), pose, stages[s]), gs.eeLink()))
                        mesh_ok[s] += mesh.pathValid(*meshes[static_cast<std::size_t>(r)], [&] { comotion::Path p; p.push_back(q); return p; }());
                }
                std::printf("  r%d g%d: grasp ik %d valid %d mesh %d | approach ik %d valid %d mesh %d%s\n", r, g,
                            raw[0], ok[0], mesh_ok[0], raw[1], ok[1], mesh_ok[1],
                            caps.canHold(r, g, pose) ? "  HOLD" : "");
            }
        }
    };
    for (std::size_t o = 0; o < scene.objects.size(); ++o) {
        const auto &obj = scene.objects[o];
        report("o" + std::to_string(o) + " start", obj.start);
        for (std::size_t p = 0; p < obj.goals.size(); ++p)
            report("o" + std::to_string(o) + " goal" + std::to_string(p), obj.goals[p]);
    }
    const auto hg = TaskHypergraph::build(caps);
    std::printf("H_T: %d vertices, %d arcs\n", hg.graph().numVertices(), hg.graph().numArcs());
    for (int a = 0; a < hg.graph().numArcs(); ++a)
        std::printf("  %s\n", hg.describeArc(a).c_str());
    return 0;
}
