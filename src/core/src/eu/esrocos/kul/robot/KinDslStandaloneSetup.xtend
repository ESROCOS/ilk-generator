/*
 * generated by Xtext 2.10.0
 */
package eu.esrocos.kul.robot


/**
 * Initialization support for running Xtext languages without Equinox extension registry.
 */
class KinDslStandaloneSetup extends KinDslStandaloneSetupGenerated {

	def static void doSetup() {
		new KinDslStandaloneSetup().createInjectorAndDoEMFRegistration()
	}
}