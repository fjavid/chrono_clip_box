#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "utils/ChUtilsCreators.h"
#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsGenerators.h"

using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

void model_box(ChSystemNSC& sys, std::shared_ptr<ChBody> floor, std::shared_ptr<ChMaterialSurface> mat,
                const int id, ChVector<>& pos, const ChQuaternion<double>& rot,
                const double box_x, const double box_y, const double box_z, const double box_t,
                const double density)
{
    auto texture = chrono_types::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("textures/concrete.jpg"));
    auto box = std::make_shared<ChBody>();
	box->SetIdentifier(id);
    
    box->GetCollisionModel()->ClearModel();
    box->SetCollide(true);
    box->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.01);
    box->GetCollisionModel()->SetDefaultSuggestedMargin(0.0001);
    double alpha = 1.1;
    box->GetCollisionModel()->AddBox(mat, box_x+2.*box_t, box_t, box_z+2.*box_t, ChVector<>(0.0, 0.5*(box_t+box_y), 0), ChQuaternion<>(1, 0, 0, 0));
    box->GetCollisionModel()->AddBox(mat, box_x+2.*box_t, box_t, box_z+2.*box_t, ChVector<>(0.0, -0.5*(box_t+box_y), 0), ChQuaternion<>(1, 0, 0, 0));
    box->GetCollisionModel()->AddBox(mat, box_x+2.*box_t, box_y+2.*box_t, box_t, ChVector<>(0.0, 0.0, 0.5*(box_t+box_z)), ChQuaternion<>(1, 0, 0, 0));
    box->GetCollisionModel()->AddBox(mat, box_x+2.*box_t, box_y+2.*box_t, box_t, ChVector<>(0.0, 0.0, -0.5*(box_t+box_z)), ChQuaternion<>(1, 0, 0, 0));
    box->GetCollisionModel()->AddBox(mat, box_t, box_y+2.*box_t, box_z+2.*box_t, ChVector<>(0.5*(box_t+box_x), 0.0, 0.0), ChQuaternion<>(1, 0, 0, 0));
    box->GetCollisionModel()->AddBox(mat, box_t, box_y+2.*box_t, box_z+2.*box_t, ChVector<>(-0.5*(box_t+box_x), 0.0, 0.0), ChQuaternion<>(1, 0, 0, 0));
    
    double volume_bottop = (box_x+2.*box_t) * (box_z+2.*box_t) * box_t;
    double volume_lefright = (box_y+2.*box_t) * (box_z+2.*box_t) * box_t;
    double volume_fronback = (box_x+2.*box_t) * (box_y+2.*box_t) * box_t;
    double volume = 2. * (volume_fronback + volume_lefright + volume_bottop);
    double mass_bottop = density * volume_bottop;
    double mass_lefright = density * volume_lefright;
    double mass_fronback = density * volume_fronback;
    double mass = 2. * (mass_fronback + mass_lefright + mass_bottop);
    double iner_xyz = mass/12.0 * ((box_y+2.*box_t)*(box_y+2.*box_t) + (box_z+2.*box_t)*(box_z+2.*box_t));
    // double iner_y = mass/12.0 * (box_x*box_x + box_z*box_z);
    // double iner_z = mass/12.0 * (box_x*box_x + box_y*box_y);
    auto box_iner = ChMatrix33<>(ChVector<>(iner_xyz, iner_xyz, iner_xyz));
    box->SetMass(mass);
    box->SetInertia(box_iner);
    // box->SetMaterialSurface(mat);
    // std::cout << "box materila is : " << box->
    box->GetCollisionModel()->BuildModel();
    sys.Add(box);
    // clip->SetPos(pos);
    // clip->SetRot(rot);
    box->AddAsset(texture);

    auto bot_viz = chrono_types::make_shared<ChBoxShape>();
    bot_viz->GetBoxGeometry().SetLengths(ChVector<>(alpha*box_x, box_t, alpha*box_z));
    bot_viz->Pos = ChVector<>(0.0, -0.5*(box_t+box_y), 0);
    box->GetAssets().push_back(bot_viz);

    auto top_viz = chrono_types::make_shared<ChBoxShape>();
    top_viz->GetBoxGeometry().SetLengths(ChVector<>(alpha*box_x, box_t, alpha*box_z));
    top_viz->Pos = ChVector<>(0.0, 0.5*(box_t+box_y), 0);
    box->GetAssets().push_back(top_viz);

    auto left_viz = chrono_types::make_shared<ChBoxShape>();
    left_viz->GetBoxGeometry().SetLengths(ChVector<>(box_t, alpha*box_y, alpha*box_z));
    left_viz->Pos = ChVector<>(-0.5*(box_t+box_x), 0.0, 0.0);
    box->GetAssets().push_back(left_viz);

    auto right_viz = chrono_types::make_shared<ChBoxShape>();
    right_viz->GetBoxGeometry().SetLengths(ChVector<>(box_t, alpha*box_y, alpha*box_z));
    right_viz->Pos = ChVector<>(0.5*(box_t+box_x), 0.0, 0.0);
    box->GetAssets().push_back(right_viz);

    // auto front_viz = chrono_types::make_shared<ChBoxShape>();
    // front_viz->GetBoxGeometry().SetLengths(ChVector<>(alpha*box_x, alpha*box_y, box_t));
    // front_viz->Pos = ChVector<>(0.0, 0.0, 0.5*(box_t+box_z));
    // box->GetAssets().push_back(front_viz);

    auto back_viz = chrono_types::make_shared<ChBoxShape>();
    back_viz->GetBoxGeometry().SetLengths(ChVector<>(alpha*box_x, alpha*box_y, box_t));
    back_viz->Pos = ChVector<>(0.0, 0.0, -0.5*(box_t+box_z));
    box->GetAssets().push_back(back_viz);
    
    auto box_motion = chrono_types::make_shared<ChLinkLockLock>();
    box_motion->Initialize(box , floor, ChCoordsys<>(ChVector<>(0, 0, 0)));

    auto mmotion_x = chrono_types::make_shared<ChFunction_Sine>(0, 2.0, 0.01);  // phase freq ampl
    box_motion->SetMotion_X(mmotion_x);
    auto mmotion_y = chrono_types::make_shared<ChFunction_Sine>(0, 2.0, 0.01);  // phase freq ampl
    box_motion->SetMotion_Y(mmotion_y);
    sys.Add(box_motion);

}

// void create_clip(ChSystemNSC& sys, std::shared_ptr<ChMaterialSurface> mat,
//                 const int id, ChVector<>& pos, const ChQuaternion<double>& rot,
//                 const double clip_w, const double clip_h, const double clip_r, const double clip_g,
//                 const double mass, const ChMatrix33<> inertia)
// {
//     auto texture = chrono_types::make_shared<ChTexture>();
//     texture->SetTextureFilename(GetChronoDataFile("textures/cubetexture_borders.png"));
//     auto clip = std::make_shared<ChBody>();
// 	clip->SetIdentifier(id);
//     clip->GetCollisionModel()->ClearModel();
//     clip->SetCollide(true);
//     clip->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.025);
//     clip->GetCollisionModel()->SetDefaultSuggestedMargin(0.0005);
//     utils::AddCylinderGeometry(clip.get(),
//                                 mat,
//                                 clip_r,
//                                 0.5*clip_w,
//                                 ChVector<>(-0.5*clip_h, 0, 0),
//                                 ChQuaternion<>(1, 0, 0, 0),
//                                 true);
//     utils::AddCylinderGeometry(clip.get(),
//                                 mat,
//                                 clip_r,
//                                 0.5*clip_w,
//                                 ChVector<>(0.5*clip_h, 0, 0),
//                                 ChQuaternion<>(1, 0, 0, 0),
//                                 true);
//     utils::AddCylinderGeometry(clip.get(),
//                                 mat,
//                                 clip_r,
//                                 0.5*clip_h,
//                                 ChVector<>(0.0, 0.5*clip_w, 0),
//                                 Q_ROTATE_Y_TO_X,
//                                 true);
//     utils::AddCylinderGeometry(clip.get(),
//                                 mat,
//                                 clip_r,
//                                 0.25*(clip_h-clip_g),
//                                 ChVector<>(-0.25*(clip_h+clip_g), -0.5*clip_w, 0),
//                                 Q_ROTATE_Y_TO_X,
//                                 true);
//     utils::AddCylinderGeometry(clip.get(),
//                                 mat,
//                                 clip_r,
//                                 0.25*(clip_h-clip_g),
//                                 ChVector<>(0.25*(clip_h+clip_g), -0.5*clip_w, 0),
//                                 Q_ROTATE_Y_TO_X,
//                                 true);
    
//     clip->SetMass(mass);
//     clip->SetInertia(inertia);
//     clip->GetCollisionModel()->BuildModel();
//     sys.Add(clip);
//     clip->SetPos(pos);
//     // clip->SetRot(rot);
//     clip->AddAsset(texture);
// }


void create_closedclip(ChSystemNSC& sys, std::shared_ptr<ChMaterialSurface> mat,
                const int id, ChVector<>& pos, const ChQuaternion<double>& rot,
                const double clip_w, const double clip_h, const double clip_r,
                const double mass, const ChVector<> inertia)
{
    // auto texture = chrono_types::make_shared<ChTexture>();
    // if (id % 2 == 0)
    //     texture->SetTextureFilename(GetChronoDataFile("textures/pink.png"));
    // else
    //     texture->SetTextureFilename(GetChronoDataFile("textures/blue.png"));
    auto clip_color = chrono_types::make_shared<ChColorAsset>(ChColor(0.8f, 0.1f, 0.1f));
    auto init_ang_velo = ChVector<>(0.0, (2.0*M_PI), 0.0);
    // auto comp_pos = ChVector<>(-0.5*clip_h, 0, 0);
    // ChQuaternion<>(1, 0, 0, 0)
    if (id % 2 != 0)
    {
        init_ang_velo = ChVector<>((2.0*M_PI), 0.0, 0.0);
        // inertia = chVector<> (inertia(0), inertia(2), inertia(1));
        clip_color = chrono_types::make_shared<ChColorAsset>(ChColor(0.1f, 0.1f, 0.8f));
    }
    

    auto clip = std::make_shared<ChBody>();
	clip->SetIdentifier(id);
    // clip->GetCollisionModel()->SetDefaultSuggestedEnvelope(0.001);
    // clip->GetCollisionModel()->SetDefaultSuggestedMargin(0.00001);
    clip->SetCollide(true);
    clip->GetCollisionModel()->ClearModel();
    // clip->SetMaterialSurface(mat);

    utils::AddCylinderGeometry(clip.get(),
                                mat,
                                clip_r,
                                0.5*clip_w,
                                ChVector<>(-0.5*clip_h, 0, 0),
                                ChQuaternion<>(1, 0, 0, 0),
                                true);
    utils::AddCylinderGeometry(clip.get(),
                                mat,
                                clip_r,
                                0.5*clip_w,
                                ChVector<>(0.5*clip_h, 0, 0),
                                ChQuaternion<>(1, 0, 0, 0),
                                true);
    utils::AddCylinderGeometry(clip.get(),
                                mat,
                                clip_r,
                                0.5*clip_h,
                                ChVector<>(0.0, 0.5*clip_w, 0),
                                Q_ROTATE_Y_TO_X,
                                true);
    utils::AddCylinderGeometry(clip.get(),
                                mat,
                                clip_r,
                                0.5*clip_h,
                                ChVector<>(0.0, -0.5*clip_w, 0),
                                Q_ROTATE_Y_TO_X,
                                true);
    
    clip->GetCollisionModel()->BuildModel();

    clip->SetMass(mass);
    clip->SetInertiaXX(inertia);
    // if (id % 2 == 0)
    // {
    //     // clip->SetPos_dt(ChVector<>(1.0, 1.0, 0.0));
    //     clip->SetWvel_par(ChVector<>(0.0, (2.0*M_PI), 0.0));
    // }
    // else
    // {
    //     // clip->SetPos_dt(ChVector<>(1.0, 1.0, 0.0));
    //     clip->SetWvel_par(ChVector<>((2.0*M_PI), 0.0, 0.0));
    // }
    clip->SetWvel_par(init_ang_velo);
    sys.Add(clip);
    clip->SetPos(pos);
    clip->SetRot(rot);
    clip->AddAsset(clip_color);
    // if (id % 2 == 0)
    //     clip->AddAsset(chrono_types::make_shared<ChColorAsset>(ChColor(0.8f, 0.1f, 0.1f)));
    // else
    //     clip->AddAsset(chrono_types::make_shared<ChColorAsset>(ChColor(0.1f, 0.1f, 0.8f)));
    
}

void create_model(ChSystemNSC& mphysicalSystem) {
    ChVector<> gravity(0, 0, 0);
    mphysicalSystem.Set_G_acc(gravity);
    SetChronoDataPath("/Users/farhad/work/cs_master/clip_entanglement/data/");
    // SetChronoDataPath("/Users/farhad/work/cs_master/chrono/data/");
    // make the shaking box
    auto floorBody = chrono_types::make_shared<ChBodyEasyBox>(1., 0.2, 1.0, 7800, false, false);
    floorBody->SetPos(ChVector<>(0, -0.3, 0));
    floorBody->SetBodyFixed(true);
    mphysicalSystem.Add(floorBody);

    double box_x = 0.25;
    double box_y = 0.25;
    double box_z = 0.25;
    double box_t = 0.01;
    int box_main_id = 9999;
    auto box_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    box_mat->SetFriction(0.2);
    // box_mat->SetCompliance(0.000001);
    box_mat->SetRestitution(0.1);
    auto box_pos = ChVector<>(0, 0, 0);
    // create_box(mphysicalSystem, floorBody, box_mat, box_main_id,
    //             box_pos, QUNIT, box_x, box_y, box_z, box_t, 7800);
    // model_box(mphysicalSystem, floorBody, box_mat, 
    //     box_main_id, box_pos, QUNIT,
    //     box_x, box_y, box_z, box_t,
    //     7800);

    auto clip_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    clip_mat->SetFriction(0.6f);
    // clip_mat->SetRollingFriction(0.001);
    // clip_mat->SetSpinningFriction(0.01);
    // clip_mat->SetCompliance(0.0001);
    // clip_mat->SetComplianceT(0.02);
    // clip_mat->SetComplianceRolling(0.08);
    // clip_mat->SetComplianceSpinning(0.08);
    // clip_mat->SetDampingF(0.001);
    clip_mat->SetRestitution(0.5f);

    auto rot = QUNIT; //Q_ROTATE_X_TO_Y; //ChQuaternion<double>(0.7565, 0, 0.6, 0.8);
    double clip_w = 0.015;
    double clip_h = 0.04;
    double clip_r = 0.001;
    double clip_g = 0.004;
    double clip_mass = 0.002471;
    auto clip_iner = ChVector<>(0.11175E-6, 0.504116E-6, 0.61463E-6);
    double cclip_mass = 0.002695;
    auto cclip_iner = ChVector<>(0.125176E-6, 0.556291236E-6, 0.680161092E-6);
    auto cclip_iner_yz = ChVector<>(0.125176E-6, 0.680161092E-6, 0.556291236E-6);
    // int idx_e = 4;
    // int idy_e = 4;
    // int idz_e = 4;
    // for (int id_x=0; id_x<idx_e; id_x++)
    //     for (int id_y=0; id_y<idy_e; id_y++)
    //         for (int id_z=0; id_z<idz_e; id_z++)
    //         {
    //             int id = id_x * idy_e * idz_e + id_y * idz_e + id_z;
    //             // std::cout << id << std::endl;
    //             auto pos = ChVector<>(id_x*1.1*clip_h-0.2*box_x, id_y*1.1*clip_w-0.2*box_y, id_z*10.0*clip_r-0.2*box_z);
    //             // create_clip(mphysicalSystem, clip_mat, id, 
    //             //         pos, rot,
    //             //         clip_w, clip_h, clip_r, clip_g,
    //             //         clip_mass, clip_iner);
    //             create_closedclip(mphysicalSystem, clip_mat, id, 
    //                     pos, rot,
    //                     clip_w, clip_h, clip_r,
    //                     clip_mass, clip_iner);
    //         }
    auto pos = ChVector<>(0, 0, 0);
    auto rot_yz = Q_ROTATE_Y_TO_Z;
    
    create_closedclip(mphysicalSystem, clip_mat, 1, 
            pos, rot,
            clip_w, clip_h, clip_r,
            cclip_mass, cclip_iner);
    pos = ChVector<>(0.5*clip_h, 0, 0);
    create_closedclip(mphysicalSystem, clip_mat, 2, 
            pos, rot_yz,
            clip_w, clip_h, clip_r,
            cclip_mass, cclip_iner_yz);
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Clip contact", core::dimension2d<u32>(800, 600));

    application.AddTypicalLights(core::vector3df(0.f, 0.f, 10.f), core::vector3df(30.f, 80.f, 60.f), 200, 10);
    application.AddTypicalCamera(core::vector3df(0.0, .05, 0.15), core::vector3df(0, -0.05, 0));

    create_model(mphysicalSystem);

    application.AssetBindAll();
    application.AssetUpdateAll();


    auto solver = chrono_types::make_shared<ChSolverPSOR>();
    solver->SetMaxIterations(200);
    // solver->SetTolerance(0.0001);
    solver->EnableWarmStart(true);
    mphysicalSystem.SetSolver(solver);

    double dT = 0.001;
    double endT = 10.0;
    int n_frames = 200;
    int f_interval = (int) (endT/dT/n_frames);
    std::vector<double> time_array;
    std::vector<double> ke_array;
    application.SetTimestep(dT);
    application.SetTryRealtime(true);
    // application.SetPOVraySave(true);
    // application.SetPOVraySaveInterval(5);
    application.SetVideoframeSave(true);
    application.SetVideoframeSaveInterval(f_interval);
    // application.GetSystem()->SetChTime(1.0);
    std::cout << "maxi recovery spped is: " << application.GetSystem()->GetMaxPenetrationRecoverySpeed() << std::endl;
    // application.GetSystem()->SetMaxPenetrationRecoverySpeed(0.3);
    // application.GetSystem()->SetMinBounceSpeed(0.6);
    double start = std::clock();
    while (application.GetDevice()->run()) {
        // std::cout << "system stepsize is : " << application.GetSystem()->GetStep() << std::endl;
        application.BeginScene(true, true, SColor(255, 140, 161, 192));
        application.DrawAll();

        application.DoStep();

        application.EndScene();
        time_array.push_back(application.GetSystem()->GetChTime());
        // ke_array.push_back(application.GetSystem()->ComputeTotalKE());
        if (application.GetSystem()->GetChTime() > endT)
            break;
    }
    double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    std::cout << "simulation run time is : " << duration << std::endl;
    return 0;
}
