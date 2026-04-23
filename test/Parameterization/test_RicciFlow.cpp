#include <catch2/catch.hpp>

#include <string>
#include <unordered_map>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <CGAL/Surface_mesh_parameterization/IO/File_off.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <Euclid/IO/OffIO.h>
#include <Euclid/IO/PlyIO.h>
#include <Euclid/MeshUtil/CGALMesh.h>
#include <Euclid/MeshUtil/PrimitiveGenerator.h>
#include <Euclid/Parameterization/SeamMesh.h>
#include <Euclid/Parameterization/RicciFlow.h>

#include <config.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;
using Vertex = typename boost::graph_traits<Mesh>::vertex_descriptor;
using Halfedge = boost::graph_traits<Mesh>::halfedge_descriptor;
using Edge = typename boost::graph_traits<Mesh>::edge_descriptor;
using Vertex_map = std::unordered_map<Vertex, bool>;
using Edge_map = std::unordered_map<Edge, bool>;
using Vertex_pmap = boost::associative_property_map<Vertex_map>;
using Edge_pmap = boost::associative_property_map<Edge_map>;
using Seam_mesh = CGAL::Seam_mesh<Mesh, Edge_pmap, Vertex_pmap>;
using UV_Map = std::unordered_map<Halfedge, Point_2>;
using UV_PMap = boost::associative_property_map<UV_Map>;
using VertexRadiusMap = std::unordered_map<Vertex, double>;
using VertexRadiusPMap = boost::associative_property_map<VertexRadiusMap>;
using EdgeWeightMap = std::unordered_map<Edge, double>;
using EdgeWeightPMap = boost::associative_property_map<EdgeWeightMap>;
using VertexCurvatureMap = std::unordered_map<Vertex, double>;
using VertexCurvaturePMap = boost::associative_property_map<VertexCurvatureMap>;

TEST_CASE("Parameterization, Ricci flow", "[parameterization][ricciflow]")
{
    Mesh mesh;
    Euclid::make_subdivision_sphere(mesh, Point_3(0, 0, 0), 1.0, 2);

    VertexRadiusMap vrm;
    VertexRadiusPMap vrpm(vrm);
    EdgeWeightMap ewm;
    EdgeWeightPMap ewpm(ewm);
    VertexCurvatureMap vcm;
    VertexCurvaturePMap vcpm(vcm);

    SECTION("circle packing metric")
    {
        Euclid::circle_packing_metric(mesh, vrpm, ewpm);

        auto v_count = static_cast<int>(std::distance(vrm.begin(), vrm.end()));
        auto e_count = static_cast<int>(std::distance(ewm.begin(), ewm.end()));

        REQUIRE(v_count == static_cast<int>(mesh.num_vertices()));
        REQUIRE(e_count == static_cast<int>(mesh.num_edges()));

        for (auto v : vertices(mesh)) {
            REQUIRE(get(vrpm, v) > 0);
        }
        for (auto e : edges(mesh)) {
            REQUIRE(get(ewpm, e) > 0);
        }
    }

    SECTION("seam mesh creation")
    {
        Edge_map emap;
        Edge_pmap epmap(emap);
        Vertex_map vmap;
        Vertex_pmap vpmap(vmap);
        Seam_mesh seam_mesh(mesh, epmap, vpmap);

        REQUIRE(CGAL::is_valid_polygon_mesh(seam_mesh));

        Euclid::mark_seam_mesh_with_cut_graph(mesh, seam_mesh);
        REQUIRE(CGAL::is_valid_polygon_mesh(seam_mesh));
    }

    SECTION("solver settings")
    {
        Euclid::RicciFlowSolverSettings s1;
        REQUIRE(s1.type == Euclid::RicciFlowSolverType::GradientDescent);
        REQUIRE(s1.max_iters == 5000);
        REQUIRE(s1.eps == 1e-5);

        Euclid::RicciFlowSolverSettings s2;
        s2.max_iters = 100;
        s2.eps = 0.1;
        s2.step = 0.1;
        s2.verbose = false;
        REQUIRE(s2.max_iters == 100);
        REQUIRE(s2.eps == 0.1);
    }
}