#version 330 core
struct Material{
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;    
    float shininess;
}; 

struct Light{
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    vec3 direction;
};

in vec3 FragPos;  
in vec3 Normal;  

out vec4 FragColor;

uniform vec3 viewPos;
uniform Material material;
uniform Light light1;
uniform Light light2;

vec3 CalcDirLight(Light light, vec3 normal, vec3 viewDir);

void main(){
    vec3 norm = normalize(Normal);
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 result = CalcDirLight(light1, norm, viewDir);
    result += CalcDirLight(light2, norm, viewDir);
    FragColor = vec4(result, 1.0);
} 

vec3 CalcDirLight(Light light, vec3 normal, vec3 viewDir){
    vec3 lightDir = normalize(-light.direction);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 ambient  = light.ambient * material.ambient;
    vec3 diffuse  = light.diffuse * (diff * material.diffuse);
    vec3 specular = light.specular * (spec * material.specular); 
    return (ambient + diffuse + specular);
}