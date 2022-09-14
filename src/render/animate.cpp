#include "animate.hpp"

void animate(Eigen::MatrixXd z, vector<double> p)
{
    sf::RenderWindow window(sf::VideoMode(1000, 1000), "SFML works!");
    sf::CircleShape shape(100.f);
    shape.setFillColor(sf::Color::Green);
    sf::RectangleShape line(sf::Vector2f(150.f, 5.f));
    sf::RectangleShape ground(sf::Vector2f(1000.f, 5.f));
    ground.setPosition(0.f, 800.f);
    sf::VertexArray line_HI(sf::LinesStrip, 2);
    sf::Vertex line_JI[2]; sf::Vertex line_KJ[2]; sf::Vertex line_KH[2];
    sf::Vertex line_OA[2]; sf::Vertex line_AB[2]; sf::Vertex line_CE[2]; 

    //line.rotate(45.f);
    line.setPosition(500.f, 500.f);

    float temp = 5.0f;
    sf::Time t1 = sf::seconds(0.01f);
    //cout<<"col "<<z.cols()<<endl;


    for (int i = 0; i < z.cols(); i++)
    {

        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }
        Eigen::VectorXd state = z.col(i);
        shape.setPosition(state(0)*100+500, 800 - state(1)*100);

        Function f_keypoints = external("keypoints", "../lib/keypoints.so"); 
        vector<double> z_vec(state.data(), state.data() + state.rows() * state.cols());
        vector<DM> arg_keypoints = {DM(z_vec), DM(p)};
        vector<DM> keypoints = f_keypoints(arg_keypoints); 
        Eigen::MatrixXd keypoints_matrix  = dmToEigen(keypoints); 
        cout<<"keypoints! \n"<<keypoints_matrix<<endl;

        line_HI[0].position = sf::Vector2f(100, 100);
        line_HI[1].position = sf::Vector2f(200, 100);

        line.rotate(temp);
        window.clear();
        window.draw(shape);
        window.draw(line);
        window.draw(line_HI);
        window.draw(ground);
        window.display();
        sf::sleep(t1);
        cout<< i<<endl;
    }
 
}


void test(){
    cout<<"hl"<<endl;
}
